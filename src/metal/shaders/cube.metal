/*
 * Metal shaders – vertex + fragment for a simple lit cube.
 *
 * Pre-compiled at build time:
 *   xcrun metal  -c cube.metal -o cube.air
 *   xcrun metallib cube.air    -o shaders.metallib
 *
 * Loaded at runtime via newLibraryWithURL:.
 */

#include <metal_stdlib>
using namespace metal;

/* ------------------------------------------------------------------ */
/* Shared types                                                        */
/* ------------------------------------------------------------------ */

struct VertexIn {
        float3 position [[attribute(0)]];
        float3 normal   [[attribute(1)]];
};

struct VertexOut {
        float4 position [[position]];
        float3 world_normal;
};

struct Uniforms {
        float4x4 model;
        float4x4 view;
        float4x4 projection;
        packed_float3 light_dir;
        float _pad0;
        packed_float3 object_color;
        float _pad1;
};

/* ------------------------------------------------------------------ */
/* Vertex shader                                                       */
/* ------------------------------------------------------------------ */

vertex VertexOut
cube_vertex(VertexIn in [[stage_in]],
            constant Uniforms &u [[buffer(1)]])
{
        VertexOut out;
        float4 world_pos = u.model * float4(in.position, 1.0);
        out.position     = u.projection * u.view * world_pos;
        out.world_normal = (u.model * float4(in.normal, 0.0)).xyz;
        return out;
}

/* ------------------------------------------------------------------ */
/* Fragment shader – ambient + Lambertian diffuse                      */
/* ------------------------------------------------------------------ */

fragment float4
cube_fragment(VertexOut in [[stage_in]],
              constant Uniforms &u [[buffer(1)]])
{
        float3 N = normalize(in.world_normal);
        float3 L = normalize(-float3(u.light_dir));

        float ambient = 0.15;
        float diffuse = max(dot(N, L), 0.0);

        float3 color = (ambient + diffuse) * float3(u.object_color);
        return float4(color, 1.0);
}
