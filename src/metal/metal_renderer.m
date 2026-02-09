/*
 * Metal renderer implementation.
 *
 * Renders a rotating lit cube using Apple's Metal API.
 * Compiled as Objective-C (.m) with ARC enabled.
 */

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>

#include "../km_mat4.h"
#include "../km_math.h"
#include "../km_geom.h"
#include "metal_renderer.h"
#include "../km_scene.h"

/* ------------------------------------------------------------------ */
/* Uniform data type (must match the Metal shader struct)               */
/* ------------------------------------------------------------------ */

struct uniforms
{
        float model[16];
        float view[16];
        float projection[16];
        struct vec3 light_dir;
        float _pad0;
        struct vec3 object_color;
        float _pad1;
};

/* ------------------------------------------------------------------ */
/* Objective-C context object (so ARC manages Metal resources)         */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Per-mesh GPU buffers                                                */
/* ------------------------------------------------------------------ */

@interface GpuMesh : NSObject
@property (nonatomic, strong) id<MTLBuffer> vertexBuffer;
@property (nonatomic, strong) id<MTLBuffer> indexBuffer;
@property (nonatomic) int indexCount;
@end

@implementation GpuMesh
@end

@interface MetalContext : NSObject

@property (nonatomic, strong) id<MTLDevice>              device;
@property (nonatomic, strong) id<MTLCommandQueue>        commandQueue;
@property (nonatomic, strong) id<MTLRenderPipelineState> pipelineState;
@property (nonatomic, strong) id<MTLDepthStencilState>   depthStencilState;
@property (nonatomic, strong) NSMutableArray<GpuMesh *> *staticGpuMeshes;
@property (nonatomic, strong) NSMutableArray<NSMutableArray<GpuMesh *> *> *entityGpuMeshes;
@property (nonatomic, strong) CAMetalLayer              *metalLayer;
@property (nonatomic)         SDL_MetalView              metalView;
@property (nonatomic, strong) id<MTLTexture>             depthTexture;

@property (nonatomic) int   width;
@property (nonatomic) int   height;

- (id<MTLTexture>)createDepthTextureWidth:(int)w height:(int)h;

@end

@implementation MetalContext

- (id<MTLTexture>)createDepthTextureWidth:(int)w height:(int)h
{
        MTLTextureDescriptor *desc = [MTLTextureDescriptor
                texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                             width:(NSUInteger)w
                                            height:(NSUInteger)h
                                         mipmapped:NO];
        desc.storageMode = MTLStorageModePrivate;
        desc.usage       = MTLTextureUsageRenderTarget;
        return [self.device newTextureWithDescriptor:desc];
}

@end

/* ------------------------------------------------------------------ */
/* Path to the pre-compiled shader library                             */
/* ------------------------------------------------------------------ */

#ifndef SHADER_LIB_PATH
#define SHADER_LIB_PATH "shaders.metallib"
#endif

/* ------------------------------------------------------------------ */
/* Renderer callbacks                                                  */
/* ------------------------------------------------------------------ */

static int metal_init(struct renderer *r, SDL_Window *window, int w, int h)
{
        (void)w;
        (void)h;

        MetalContext *ctx = [[MetalContext alloc] init];
        if (!ctx) return -1;

        /* ---- SDL Metal view / layer ---- */
        ctx.metalView = SDL_Metal_CreateView(window);
        if (!ctx.metalView) {
                fprintf(stderr, "SDL_Metal_CreateView: %s\n", SDL_GetError());
                return -1;
        }

        ctx.metalLayer = (__bridge CAMetalLayer *)SDL_Metal_GetLayer(
                                                        ctx.metalView);
        if (!ctx.metalLayer) {
                fprintf(stderr, "Failed to get CAMetalLayer\n");
                SDL_Metal_DestroyView(ctx.metalView);
                return -1;
        }

        /* ---- Device & command queue ---- */
        ctx.device = MTLCreateSystemDefaultDevice();
        if (!ctx.device) {
                fprintf(stderr, "Metal is not supported on this system\n");
                SDL_Metal_DestroyView(ctx.metalView);
                return -1;
        }

        ctx.metalLayer.device      = ctx.device;
        ctx.metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
        ctx.commandQueue           = [ctx.device newCommandQueue];

        /* Actual drawable size (Retina-aware) */
        CGSize ds = ctx.metalLayer.drawableSize;
        ctx.width  = (int)ds.width;
        ctx.height = (int)ds.height;

        /* ---- Load pre-compiled shader library ---- */
        NSError *err = nil;
        NSURL *libURL = [NSURL fileURLWithPath:
                         @SHADER_LIB_PATH];
        id<MTLLibrary> library = [ctx.device newLibraryWithURL:libURL
                                                         error:&err];
        if (!library) {
                fprintf(stderr, "Failed to load %s: %s\n",
                        SHADER_LIB_PATH,
                        [[err localizedDescription] UTF8String]);
                SDL_Metal_DestroyView(ctx.metalView);
                return -1;
        }

        id<MTLFunction> vertFunc = [library newFunctionWithName:@"cube_vertex"];
        id<MTLFunction> fragFunc = [library newFunctionWithName:@"cube_fragment"];

        /* ---- Vertex descriptor ---- */
        MTLVertexDescriptor *vd = [[MTLVertexDescriptor alloc] init];
        /* position: float3 at offset 0 */
        vd.attributes[0].format      = MTLVertexFormatFloat3;
        vd.attributes[0].offset      = 0;
        vd.attributes[0].bufferIndex = 0;
        /* normal:   float3 at offset 12 */
        vd.attributes[1].format      = MTLVertexFormatFloat3;
        vd.attributes[1].offset      = 12;
        vd.attributes[1].bufferIndex = 0;
        vd.layouts[0].stride         = 24;
        vd.layouts[0].stepFunction   = MTLVertexStepFunctionPerVertex;

        /* ---- Render pipeline ---- */
        MTLRenderPipelineDescriptor *pd =
                [[MTLRenderPipelineDescriptor alloc] init];
        pd.vertexFunction                  = vertFunc;
        pd.fragmentFunction                = fragFunc;
        pd.vertexDescriptor                = vd;
        pd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        pd.depthAttachmentPixelFormat      = MTLPixelFormatDepth32Float;

        ctx.pipelineState = [ctx.device
                newRenderPipelineStateWithDescriptor:pd error:&err];
        if (!ctx.pipelineState) {
                fprintf(stderr, "Pipeline state error: %s\n",
                        [[err localizedDescription] UTF8String]);
                SDL_Metal_DestroyView(ctx.metalView);
                return -1;
        }

        /* ---- Depth / stencil state ---- */
        MTLDepthStencilDescriptor *dsd =
                [[MTLDepthStencilDescriptor alloc] init];
        dsd.depthCompareFunction = MTLCompareFunctionLess;
        dsd.depthWriteEnabled    = YES;
        ctx.depthStencilState    = [ctx.device
                newDepthStencilStateWithDescriptor:dsd];

        /* ---- Mesh arrays (populated later via upload) ---- */
        ctx.staticGpuMeshes = [[NSMutableArray alloc] init];
        ctx.entityGpuMeshes = [[NSMutableArray alloc] init];

        /* ---- Depth texture ---- */
        ctx.depthTexture = [ctx createDepthTextureWidth:ctx.width
                                                 height:ctx.height];

        /* Transfer ownership to the C struct */
        r->ctx = (__bridge_retained void *)ctx;

        fprintf(stdout, "Metal renderer initialised (%d×%d)\n",
                ctx.width, ctx.height);
        return 0;
}

/* ------------------------------------------------------------------ */
/* Upload meshes to GPU                                                */
/* ------------------------------------------------------------------ */

static GpuMesh *upload_one_mesh(id<MTLDevice> device, const struct mesh *m)
{
        GpuMesh *gm = [[GpuMesh alloc] init];

        NSUInteger vsize = m->vertex_count * sizeof(struct vertex);
        gm.vertexBuffer = [device
                newBufferWithBytes:m->vertices
                            length:vsize
                           options:MTLResourceStorageModeShared];

        NSUInteger isize = m->index_count * sizeof(uint16_t);
        gm.indexBuffer = [device
                newBufferWithBytes:m->indices
                            length:isize
                           options:MTLResourceStorageModeShared];

        gm.indexCount = (int)m->index_count;
        return gm;
}

static int metal_upload(struct renderer *r,
                        const struct mesh *static_meshes, int static_count,
                        const struct entity *entities, int entity_count)
{
        MetalContext *ctx = (__bridge MetalContext *)r->ctx;

        /* Discard previously uploaded meshes */
        [ctx.staticGpuMeshes removeAllObjects];
        [ctx.entityGpuMeshes removeAllObjects];

        /* Upload static meshes (world surfaces) */
        for (int i = 0; i < static_count; i++) {
                GpuMesh *gm = upload_one_mesh(ctx.device, &static_meshes[i]);
                [ctx.staticGpuMeshes addObject:gm];
        }

        /* Upload entity meshes (one group per entity) */
        for (int i = 0; i < entity_count; i++) {
                const struct entity *e = &entities[i];
                NSMutableArray<GpuMesh *> *group = [[NSMutableArray alloc] init];
                for (int j = 0; j < e->surface_count; j++) {
                        GpuMesh *gm = upload_one_mesh(ctx.device,
                                                      &e->surfaces[j]);
                        [group addObject:gm];
                }
                [ctx.entityGpuMeshes addObject:group];
        }

        fprintf(stdout, "Uploaded %d static + %d entity mesh group(s) to GPU\n",
                static_count, entity_count);
        return 0;
}

/* ------------------------------------------------------------------ */
/* Render                                                              */
/* ------------------------------------------------------------------ */

static void draw_mesh(id<MTLRenderCommandEncoder> enc,
                      GpuMesh *gm,
                      const struct uniforms *u)
{
        [enc setVertexBuffer:gm.vertexBuffer offset:0 atIndex:0];
        [enc setVertexBytes:u length:sizeof(*u) atIndex:1];
        [enc setFragmentBytes:u length:sizeof(*u) atIndex:1];

        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:(NSUInteger)gm.indexCount
                         indexType:MTLIndexTypeUInt16
                       indexBuffer:gm.indexBuffer
                 indexBufferOffset:0];
}

static void metal_render(struct renderer *r, struct scene* scene, float dt)
{
        @autoreleasepool {
                MetalContext *ctx = (__bridge MetalContext *)r->ctx;
                (void)dt;

                /* ---- Shared view / projection ---- */
                struct uniforms u;

                mat4_look_at(u.view,
                             &scene->cam.pos,
                             &scene->cam.center,
                             &scene->cam.up);

                float aspect = (float)ctx.width / (float)ctx.height;
                mat4_perspective(u.projection, 1.0472f, aspect,
                                 0.1f, 100.0f);       /* 60° FOV */

                /* Light direction (travels from upper-right toward origin) */
                u.light_dir.x = -0.4082f;
                u.light_dir.y = -0.8165f;
                u.light_dir.z = -0.4082f;
                u._pad0        =  0.0f;

                /* Object colour – pleasant blue */
                u.object_color.x = 0.3f;
                u.object_color.y = 0.6f;
                u.object_color.z = 0.8f;
                u._pad1           = 0.0f;

                /* ---- Acquire drawable ---- */
                id<CAMetalDrawable> drawable =
                        [ctx.metalLayer nextDrawable];
                if (!drawable) return;

                /* ---- Render pass ---- */
                MTLRenderPassDescriptor *rpd =
                        [MTLRenderPassDescriptor renderPassDescriptor];
                rpd.colorAttachments[0].texture     = drawable.texture;
                rpd.colorAttachments[0].loadAction  = MTLLoadActionClear;
                rpd.colorAttachments[0].storeAction = MTLStoreActionStore;
                rpd.colorAttachments[0].clearColor  =
                        MTLClearColorMake(0.15, 0.15, 0.18, 1.0);
                rpd.depthAttachment.texture     = ctx.depthTexture;
                rpd.depthAttachment.loadAction  = MTLLoadActionClear;
                rpd.depthAttachment.storeAction = MTLStoreActionDontCare;
                rpd.depthAttachment.clearDepth  = 1.0;

                id<MTLCommandBuffer> cmd = [ctx.commandQueue commandBuffer];
                id<MTLRenderCommandEncoder> enc =
                        [cmd renderCommandEncoderWithDescriptor:rpd];

                [enc setRenderPipelineState:ctx.pipelineState];
                [enc setDepthStencilState:ctx.depthStencilState];
                [enc setFrontFacingWinding:MTLWindingCounterClockwise];
                [enc setCullMode:MTLCullModeBack];

                MTLViewport vp = {
                        0.0, 0.0,
                        (double)ctx.width, (double)ctx.height,
                        0.0, 1.0
                };
                [enc setViewport:vp];

                /* ---- Draw static meshes (identity model matrix) ---- */
                mat4_identity(u.model);
                for (GpuMesh *gm in ctx.staticGpuMeshes) {
                        draw_mesh(enc, gm, &u);
                }

                /* ---- Draw entities (per-entity transform) ---- */
                for (int i = 0; i < scene->entity_count; i++) {
                        const struct entity *e = &scene->entities[i];
                        const struct particle *p = &e->o.p;

                        /* Model = Translate * Rz * Ry * Rx */
                        float t[16], rx[16], ry[16], rz[16], tmp[16];
                        mat4_translate(t, p->p.x, p->p.y, p->p.z);
                        mat4_rotate_x(rx, p->r.x);
                        mat4_rotate_y(ry, p->r.y);
                        mat4_rotate_z(rz, p->r.z);

                        mat4_multiply(tmp, ry, rx);     /* tmp = Ry * Rx */
                        mat4_multiply(tmp, rz, tmp);    /* tmp = Rz * Ry * Rx */
                        mat4_multiply(u.model, t, tmp); /* model = T * R */

                        NSMutableArray<GpuMesh *> *group =
                                ctx.entityGpuMeshes[(NSUInteger)i];
                        for (GpuMesh *gm in group) {
                                draw_mesh(enc, gm, &u);
                        }
                }

                [enc endEncoding];
                [cmd presentDrawable:drawable];
                [cmd commit];
        }
}

static void metal_resize(struct renderer *r, int width, int height)
{
        MetalContext *ctx = (__bridge MetalContext *)r->ctx;
        (void)width;
        (void)height;

        CGSize ds = ctx.metalLayer.drawableSize;
        ctx.width  = (int)ds.width;
        ctx.height = (int)ds.height;
        ctx.depthTexture = [ctx createDepthTextureWidth:ctx.width
                                                 height:ctx.height];

        fprintf(stdout, "Renderer resized to %d×%d\n",
                ctx.width, ctx.height);
}

static void metal_cleanup(struct renderer *r)
{
        if (r->ctx) {
                /* __bridge_transfer takes ownership back so ARC releases */
                MetalContext *ctx =
                        (__bridge_transfer MetalContext *)r->ctx;
                if (ctx.metalView)
                        SDL_Metal_DestroyView(ctx.metalView);
                r->ctx = NULL;
        }
}

struct renderer* metal_renderer_create(void)
{
        struct renderer *r = calloc(1, sizeof(*r));
        if (!r)
        {
                return NULL;
        }

        r->init    = metal_init;
        r->upload  = metal_upload;
        r->render  = metal_render;
        r->resize  = metal_resize;
        r->cleanup = metal_cleanup;
        r->ctx     = NULL;

        return r;
}
