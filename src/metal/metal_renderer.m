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
#include "metal_renderer.h"

/* ------------------------------------------------------------------ */
/* Vertex / uniform data types (must match the Metal shader structs)   */
/* ------------------------------------------------------------------ */

struct vertex
{
        float position[3];
        float normal[3];
};

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
/* Cube geometry – 24 vertices (4 per face), 36 indices                */
/* ------------------------------------------------------------------ */

static const struct vertex cube_vertices[] =
{
        /* Front face  (z = +1)  normal ( 0,  0,  1) */
        {{ -1, -1,  1 }, {  0,  0,  1 }},
        {{  1, -1,  1 }, {  0,  0,  1 }},
        {{  1,  1,  1 }, {  0,  0,  1 }},
        {{ -1,  1,  1 }, {  0,  0,  1 }},
        /* Back face   (z = -1)  normal ( 0,  0, -1) */
        {{  1, -1, -1 }, {  0,  0, -1 }},
        {{ -1, -1, -1 }, {  0,  0, -1 }},
        {{ -1,  1, -1 }, {  0,  0, -1 }},
        {{  1,  1, -1 }, {  0,  0, -1 }},
        /* Top face    (y = +1)  normal ( 0,  1,  0) */
        {{ -1,  1,  1 }, {  0,  1,  0 }},
        {{  1,  1,  1 }, {  0,  1,  0 }},
        {{  1,  1, -1 }, {  0,  1,  0 }},
        {{ -1,  1, -1 }, {  0,  1,  0 }},
        /* Bottom face (y = -1)  normal ( 0, -1,  0) */
        {{ -1, -1, -1 }, {  0, -1,  0 }},
        {{  1, -1, -1 }, {  0, -1,  0 }},
        {{  1, -1,  1 }, {  0, -1,  0 }},
        {{ -1, -1,  1 }, {  0, -1,  0 }},
        /* Right face  (x = +1)  normal ( 1,  0,  0) */
        {{  1, -1,  1 }, {  1,  0,  0 }},
        {{  1, -1, -1 }, {  1,  0,  0 }},
        {{  1,  1, -1 }, {  1,  0,  0 }},
        {{  1,  1,  1 }, {  1,  0,  0 }},
        /* Left face   (x = -1)  normal (-1,  0,  0) */
        {{ -1, -1, -1 }, { -1,  0,  0 }},
        {{ -1, -1,  1 }, { -1,  0,  0 }},
        {{ -1,  1,  1 }, { -1,  0,  0 }},
        {{ -1,  1, -1 }, { -1,  0,  0 }},
};

static const uint16_t cube_indices[] =
{
         0,  1,  2,   0,  2,  3,   /* Front  */
         4,  5,  6,   4,  6,  7,   /* Back   */
         8,  9, 10,   8, 10, 11,   /* Top    */
        12, 13, 14,  12, 14, 15,   /* Bottom */
        16, 17, 18,  16, 18, 19,   /* Right  */
        20, 21, 22,  20, 22, 23,   /* Left   */
};

/* ------------------------------------------------------------------ */
/* Objective-C context object (so ARC manages Metal resources)         */
/* ------------------------------------------------------------------ */

@interface MetalContext : NSObject

@property (nonatomic, strong) id<MTLDevice>              device;
@property (nonatomic, strong) id<MTLCommandQueue>        commandQueue;
@property (nonatomic, strong) id<MTLRenderPipelineState> pipelineState;
@property (nonatomic, strong) id<MTLDepthStencilState>   depthStencilState;
@property (nonatomic, strong) id<MTLBuffer>              vertexBuffer;
@property (nonatomic, strong) id<MTLBuffer>              indexBuffer;
@property (nonatomic, strong) CAMetalLayer              *metalLayer;
@property (nonatomic)         SDL_MetalView              metalView;
@property (nonatomic, strong) id<MTLTexture>             depthTexture;

@property (nonatomic) int   width;
@property (nonatomic) int   height;
@property (nonatomic) float rotation;
@property (nonatomic) int   numIndices;

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

        /* ---- Geometry buffers ---- */
        ctx.vertexBuffer = [ctx.device
                newBufferWithBytes:cube_vertices
                            length:sizeof(cube_vertices)
                           options:MTLResourceStorageModeShared];
        ctx.indexBuffer  = [ctx.device
                newBufferWithBytes:cube_indices
                            length:sizeof(cube_indices)
                           options:MTLResourceStorageModeShared];
        ctx.numIndices   = sizeof(cube_indices) / sizeof(cube_indices[0]);

        /* ---- Depth texture ---- */
        ctx.depthTexture = [ctx createDepthTextureWidth:ctx.width
                                                 height:ctx.height];

        ctx.rotation = 0.0f;

        /* Transfer ownership to the C struct */
        r->ctx = (__bridge_retained void *)ctx;

        fprintf(stdout, "Metal renderer initialised (%d×%d)\n",
                ctx.width, ctx.height);
        return 0;
}

static void metal_render(struct renderer *r, float dt)
{
        @autoreleasepool {
                MetalContext *ctx = (__bridge MetalContext *)r->ctx;

                /* ---- Update rotation ---- */
                ctx.rotation += dt * 1.0f;   /* 1 rad / s */

                /* ---- Build uniforms ---- */
                struct uniforms u;
                float ry[16], rx[16];

                mat4_rotate_y(ry, ctx.rotation);
                mat4_rotate_x(rx, 0.4363f);          /* ≈ 25° tilt */
                mat4_multiply(u.model, rx, ry);

                struct vec3 eye    = { .a = { 0.0f, 2.0f, 5.0f } };
                struct vec3 center = { .a = { 0.0f, 0.0f, 0.0f } };
                struct vec3 up     = { .a = { 0.0f, 1.0f, 0.0f } };
                mat4_look_at(u.view, &eye, &center, &up);

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

                [enc setVertexBuffer:ctx.vertexBuffer offset:0 atIndex:0];
                [enc setVertexBytes:&u length:sizeof(u) atIndex:1];
                [enc setFragmentBytes:&u length:sizeof(u) atIndex:1];

                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:(NSUInteger)ctx.numIndices
                                 indexType:MTLIndexTypeUInt16
                               indexBuffer:ctx.indexBuffer
                         indexBufferOffset:0];

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

/* ------------------------------------------------------------------ */
/* Public factory function                                             */
/* ------------------------------------------------------------------ */

struct renderer* metal_renderer_create(void)
{
        struct renderer *r = calloc(1, sizeof(*r));
        if (!r) return NULL;

        r->init    = metal_init;
        r->render  = metal_render;
        r->resize  = metal_resize;
        r->cleanup = metal_cleanup;
        r->ctx     = NULL;

        return r;
}
