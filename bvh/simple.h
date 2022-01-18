//
// Embree implementation for comparisons
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include "bvh/base.h"
#include "bvh/util.h"

#include <cmath>
#include <stack>
#include <vector>


namespace bvh
{
    class Simple : public Base
    {
    public:
        void build(const Mesh* mesh) final
        {
            // We know how many triangles there will be, so reserve some space
            _triangles.reserve(mesh->triangles);


            // Find initial volume
            std::vector<Prim> prims;
            prims.reserve(mesh->triangles);

            Volume vol;
            vol.first = 0;
            vol.last  = mesh->triangles;

            for (uint32_t ii = 0; ii < mesh->triangles; ii++)
            {
                v3 v0(mesh->positions + 3 * mesh->indices[3 * ii + 0]);
                v3 v1(mesh->positions + 3 * mesh->indices[3 * ii + 1]);
                v3 v2(mesh->positions + 3 * mesh->indices[3 * ii + 2]);

                Prim prim;
                prim.mid   = (v0 + v1 + v2) / 3.0f;
                prim.index = ii;

                prims.push_back(prim);

                vol.min = min(vol.min, prim.mid);
                vol.max = max(vol.max, prim.mid);
            }


            // Split each pending volume
            const uint32_t MAX_NODE_SIZE = 4;

            std::stack<Volume> volumes;
            for (;;)
            {
                uint32_t node_index = static_cast<uint32_t>(_nodes.size());
                _nodes.push_back(Node{});

                // Set parent 'right' index if required
                if (vol.parent != INVALID)
                    _nodes[vol.parent].offset = node_index;


                uint32_t count = vol.last - vol.first;
                if (count > MAX_NODE_SIZE)
                {
                    // Always split along largest axis
                    v3       delta = vol.max - vol.min;
                    uint32_t axis  = max_dim(delta);
                    float    split = 0.5f * (vol.min[axis] + vol.max[axis]);

                    _nodes[node_index].axis = (uint16_t)(axis);


                    // Partition primitives
                    Volume left;
                    Volume right;

                    uint32_t l = vol.first;
                    uint32_t r = vol.last;

                    while (l < r)
                    {
                        if (prims[l].mid[axis] < split)
                        {
                            // Goes on left
                            left.min = min(left.min, prims[l].mid);
                            left.max = max(left.max, prims[l].mid);

                            l++;
                        }
                        else
                        {
                            // Goes on right
                            right.min = min(right.min, prims[l].mid);
                            right.max = max(right.max, prims[l].mid);

                            --r;
                            std::swap(prims[l], prims[r]);
                        }
                    }

                    if (l == vol.first || l == vol.last)
                    {
                        // Can't split - just do it arbitrarily
                        l = (vol.first + vol.last) / 2;

                        left.min  = vol.min;
                        left.max  = vol.max;
                        right.min = vol.min;
                        right.max = vol.max;
                    }

                    left.first = vol.first;
                    left.last  = l;

                    right.first  = l;
                    right.last   = vol.last;
                    right.parent = node_index;


                    // Process the left child next, push the right child on the stack for later
                    vol = left;
                    volumes.push(right);
                }
                else
                {
                    // Add triangles for this node and set its bounds
                    _nodes[node_index].offset = static_cast<uint32_t>(_triangles.size());
                    _nodes[node_index].count  = count;

                    for (uint32_t ii = vol.first; ii < vol.last; ii++)
                    {
                        uint32_t tri_idx = prims[ii].index;

                        v3 v0(mesh->positions + 3 * mesh->indices[3 * tri_idx + 0]);
                        v3 v1(mesh->positions + 3 * mesh->indices[3 * tri_idx + 1]);
                        v3 v2(mesh->positions + 3 * mesh->indices[3 * tri_idx + 2]);

                        Triangle tri;
                        tri.p0 = v0;
                        tri.p1 = v1;
                        tri.p2 = v2;

                        tri.index = tri_idx;

                        _triangles.push_back(tri);

                        _nodes[node_index].min = min(min(_nodes[node_index].min, v0), min(v1, v2));
                        _nodes[node_index].max = max(max(_nodes[node_index].max, v0), max(v1, v2));
                    }


                    // Pop the next node to process off the stack.  If it's empty, we're done
                    if (volumes.empty())
                        break;

                    vol = volumes.top();
                    volumes.pop();
                }
            }


            // Propagate bounds from children to parent nodes
            // We know parents always appear in the node list before children, so just walk backwards through the array
            uint32_t idx = static_cast<uint32_t>(_nodes.size());
            while (idx > 0)
            {
                idx--;

                if (_nodes[idx].count == 0)
                {
                    uint32_t left  = idx + 1;
                    uint32_t right = _nodes[idx].offset;

                    _nodes[idx].min = min(_nodes[left].min, _nodes[right].min);
                    _nodes[idx].max = max(_nodes[left].max, _nodes[right].max);
                }
            }
        }


        void trace(uint32_t rays, Ray* input, Hit* output, uint32_t flags) final
        {
            std::vector<uint32_t> stack;
            stack.reserve(128);

            for (uint32_t ii = 0; ii < rays; ii++)
            {
                Hit hit;
                hit.triangle = TRIANGLE_INVALID;

                v3    org(input[ii].origin);
                v3    dir(input[ii].direction);
                float min_t = input[ii].min_t;
                float max_t = input[ii].max_t;

                v3 inv_dir(1.0f / dir[0], 1.0f / dir[1], 1.0f / dir[2]);

                WoopRay wr = woop_ray(org, dir);

                uint32_t node_index = 0;
                for (;;)
                {
                    v3 min = _nodes[node_index].min;
                    v3 max = _nodes[node_index].max;

                    if (ray_vs_bounds(org, inv_dir, min_t, max_t, min, max))
                    {
                        uint32_t count  = _nodes[node_index].count;
                        uint32_t offset = _nodes[node_index].offset;
                        if (count == 0)
                        {
                            // Descend to children (decide which node to visit first based on ray direction)
                            uint32_t axis = _nodes[node_index].axis;
                            if (dir[axis] > 0.0f)
                            {
                                stack.push_back(offset);
                                node_index++;
                            }
                            else
                            {
                                stack.push_back(node_index + 1);
                                node_index = offset;
                            }
                            continue;
                        }


                        // Test triangles
                        for (uint32_t jj = 0; jj < count; jj++)
                        {
                            const auto& tri = _triangles[offset + jj];

                            if (woop_ray_vs_triangle(wr, min_t, max_t, tri.p0, tri.p1, tri.p2, hit.barycentric, &max_t))
                            {
                                hit.triangle = tri.index;
                                if (flags & TRACE_SHADOW)
                                {
                                    stack.clear();
                                    break;
                                }
                            }
                        }
                    }

                    if (stack.empty())
                        break;

                    node_index = stack[stack.size() - 1];
                    stack.pop_back();
                }

                output[ii] = hit;
            }
        }


    private:
        static const uint32_t INVALID = static_cast<uint32_t>(-1);

        struct Prim
        {
            v3       mid;
            uint32_t index;
        };

        struct Volume
        {
            uint32_t first  = 0;
            uint32_t last   = 0;
            uint32_t parent = INVALID;
            v3       min    = {+INFINITY, +INFINITY, +INFINITY};
            v3       max    = {-INFINITY, -INFINITY, -INFINITY};
        };

        struct Node
        {
            v3       min    = {+INFINITY, +INFINITY, +INFINITY};
            v3       max    = {-INFINITY, -INFINITY, -INFINITY};
            uint32_t offset = 0;
            uint16_t count  = 0;
            uint16_t axis   = 0;
        };

        struct Triangle
        {
            v3       p0;
            v3       p1;
            v3       p2;
            uint32_t index;
        };

        std::vector<Node>     _nodes;
        std::vector<Triangle> _triangles;
    };
}
