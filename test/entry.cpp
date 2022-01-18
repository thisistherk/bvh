//
// Test entry point
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#include "ambient.h"
#include "log.h"
#include "mesh.h"
#include "timer.h"
#include "v3.h"

#include "bvh/base.h"
#include "bvh/embree.h"
#include "bvh/simple.h"

#include <cstdint>
#include <memory>


namespace
{
    constexpr float DEGREES_TO_RADIANS = 0.01745329251f;


    // Selection of scene and camera definitions

    struct Config
    {
        const char* name;
        const char* file;
        uint32_t    axis;
        bool        has_camera  = false;
        v3          camera_from = v3();
        v3          camera_to   = v3();
        v3          camera_up   = v3();
    };

    const Config configs[] = {
        {"hairball", DIR_SOLIDS "/hairball.obj", 1},
        {"buddha", DIR_SOLIDS "/buddha.obj", 1},
        {"bunny", DIR_SOLIDS "/bunny.obj", 1},
        {"cube", DIR_SOLIDS "/cube.obj", 1},
        {"sanmiguel1",
         DIR_SOLIDS "/san-miguel.obj",
         1,
         true,
         v3(26.6878f, 7.31451f, -2.71626f),
         v3(25.8663f, 7.37751f, -2.14962f),
         v3(0.0518586f, 0.998014f, -0.0357671f)},
        {"sanmiguel2",
         DIR_SOLIDS "/san-miguel.obj",
         1,
         true,
         v3(26.2755f, 7.15164f, 4.93625f),
         v3(25.7736f, 7.19675f, 4.07249f),
         v3(0.02266f, 0.998982f, 0.0389986f)},
        {"sanmiguel3",
         DIR_SOLIDS "/san-miguel.obj",
         1,
         true,
         v3(22.8676f, 1.94784f, 12.9289f),
         v3(22.2116f, 2.04755f, 12.1807f),
         v3(0.065738f, 0.995016f, 0.0749807f)},
        {"sanmiguel4",
         DIR_SOLIDS "/san-miguel.obj",
         1,
         true,
         v3(6.37319f, 1.53861f, 5.62511f),
         v3(7.09618f, 1.64532f, 4.94254f),
         v3(-0.0775949f, 0.99429f, 0.0732566f)},
    };
    int num_configs = sizeof(configs) / sizeof(configs[0]);


    // Find a scene config with a given name

    const Config* find_config(const char* name)
    {
        int idx = 0;
        while (idx < num_configs)
        {
            const Config* config = configs + idx;
            if (std::strcmp(config->name, name) == 0)
                return config;

            idx++;
        }

        return nullptr;
    }


    // Create a BVH provider with the given name

    std::unique_ptr<bvh::Base> create_bvh(const char* name)
    {
        std::unique_ptr<bvh::Base> bvh;

        if (std::strcmp(name, "simple") == 0)
            bvh = std::make_unique<bvh::Simple>();
#if defined(BVH_EMBREE)
        else if (std::strcmp(name, "embree") == 0)
            bvh = std::make_unique<bvh::Embree>();
        else if (std::strcmp(name, "embree_low") == 0)
            bvh = std::make_unique<bvh::Embree>(RTC_BUILD_QUALITY_LOW);
        else if (std::strcmp(name, "embree_medium") == 0)
            bvh = std::make_unique<bvh::Embree>(RTC_BUILD_QUALITY_MEDIUM);
        else if (std::strcmp(name, "embree_high") == 0)
            bvh = std::make_unique<bvh::Embree>(RTC_BUILD_QUALITY_HIGH);
#endif

        return bvh;
    }

}


int main(int argc, const char* argv[])
{
    // Parse command line to find scene/BVH provider
    if (argc != 3)
        error("Usage: %s <config> <bvh type>", argv[0]);

    auto config = find_config(argv[1]);
    if (!config)
        error("Config '%s' not found", argv[1]);

    auto bvh = create_bvh(argv[2]);
    if (!bvh)
        error("BVH '%s' not found", argv[2]);


    // Read mesh
    Timer time_read;
    time_read.begin();

    Mesh mesh(config->file);
    if (mesh.triangles() == 0)
        error("Failed to read mesh: %s", config->file);

    time_read.end();
    log("Read %d triangles in %0.2fs", mesh.triangles(), time_read.seconds());


    // Add a ground plane to the mesh
    constexpr float plane_size = 5.0f;
    mesh.add_plane(config->axis, plane_size);


    // Generate a camera
    Camera camera;
    if (config->has_camera)
    {
        camera.to   = config->camera_to;
        camera.from = config->camera_from;
        camera.up   = config->camera_up;
    }
    else
    {
        camera.to               = mesh.centre();
        camera.from             = camera.to + 3.0f * mesh.radius() * v3(0.2f, 0.3f, 0.4f);
        camera.up               = v3(0.0f, 0.0f, 0.0f);
        camera.up[config->axis] = 1.0f;
    }

    camera.fov = DEGREES_TO_RADIANS * 90.0f;


    // Build BVH
    Timer time_build;
    time_build.begin();

    bvh::Mesh bvh_mesh;
    bvh_mesh.vertices  = mesh.vertices();
    bvh_mesh.triangles = mesh.triangles();
    bvh_mesh.positions = reinterpret_cast<const float*>(mesh.positions());
    bvh_mesh.indices   = mesh.indices();

    bvh->build(&bvh_mesh);

    time_build.end();
    log("Built BVH type '%s' in %0.2fs", argv[2], time_build.seconds());


    // Render an image
    constexpr uint32_t SAMPLES = 16;
    constexpr uint32_t WIDTH   = 1920;
    constexpr uint32_t HEIGHT  = 1080;

    log("Rendering %dx%d Ambient Occlusion image with %d samples per pixel", WIDTH, HEIGHT, SAMPLES);

    Timer time_render;
    time_render.begin();

    AmbientOcclusion ao;
    ao.begin(&mesh, bvh.get(), WIDTH, HEIGHT, camera);

    for (uint32_t ii = 0; ii < SAMPLES; ii++)
        ao.refine();

    time_render.end();

    log("Rendered in %0.2fs", time_render.seconds());


    // Output stats
    const auto& stats = ao.stats();
    log("Traced:");
    log("   %d primary rays in %0.2fs (%0.2f Mrays/s)",
        stats->primary_count,
        stats->primary_timer.seconds(),
        static_cast<float>(stats->primary_count) * 1.0e-6f / stats->primary_timer.seconds());
    log("   %d shadow rays in %0.2fs (%0.2f Mrays/s)",
        stats->shadow_count,
        stats->shadow_timer.seconds(),
        static_cast<float>(stats->shadow_count) * 1.0e-6f / stats->shadow_timer.seconds());


    // Write to file
    ao.image()->write("output.bmp");

    return 0;
}
