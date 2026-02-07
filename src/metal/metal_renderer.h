/*
 * Metal renderer – public header.
 */

#ifndef KM_METAL_RENDERER_H
#define KM_METAL_RENDERER_H

#include "../km_renderer.h"

/**
 * Create a renderer that uses Apple Metal for drawing.
 * The caller must eventually call renderer->cleanup(renderer) and free().
 */
extern struct renderer *metal_renderer_create(void);

#endif /* KM_METAL_RENDERER_H */
