/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BKE_mesh.hh"
#include "BKE_subdiv.hh"
#include "BKE_subdiv_mesh.hh"

#include "bmesh.hh"
#include "bmesh_tools.hh"
#include "intern/bmesh_operators.hh"
#include "intern/bmesh_marking.hh"

#include "GEO_randomize.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_subdivide_axis_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_output<decl::Geometry>("Mesh").propagate_all().align_with_previous();
  b.add_input<decl::Int>("Level X").default_value(0).min(0).max(10).description(
      "Subdivision level along X axis");
  b.add_input<decl::Int>("Level Y").default_value(0).min(0).max(10).description(
      "Subdivision level along Y axis");
  b.add_input<decl::Int>("Level Z").default_value(0).min(0).max(10).description(
      "Subdivision level along Z axis");
}

#ifdef WITH_OPENSUBDIV
static Mesh *axis_subdivide_mesh(const Mesh &mesh,
                                  const int level_x,
                                  const int level_y,
                                  const int level_z)
{
  /* If all levels are the same, use the simple subdivide method. */
  if (level_x == level_y && level_y == level_z) {
    if (level_x == 0) {
      return nullptr;
    }
    /* Initialize mesh settings. */
    bke::subdiv::ToMeshSettings mesh_settings;
    mesh_settings.resolution = (1 << level_x) + 1;
    mesh_settings.use_optimal_display = false;

    /* Initialize subdivision settings. */
    bke::subdiv::Settings subdiv_settings;
    subdiv_settings.is_simple = true;
    subdiv_settings.is_adaptive = false;
    subdiv_settings.use_creases = false;
    subdiv_settings.level = 1;
    subdiv_settings.vtx_boundary_interpolation =
        bke::subdiv::vtx_boundary_interpolation_from_subsurf(0);
    subdiv_settings.fvar_linear_interpolation = bke::subdiv::fvar_interpolation_from_uv_smooth(0);

    /* Apply subdivision from mesh. */
    bke::subdiv::Subdiv *subdiv = bke::subdiv::new_from_mesh(&subdiv_settings, &mesh);
    if (!subdiv) {
      return nullptr;
    }

    Mesh *result = bke::subdiv::subdiv_to_mesh(subdiv, &mesh_settings, &mesh);
    bke::subdiv::free(subdiv);
    geometry::debug_randomize_mesh_order(result);
    return result;
  }

  /* For different levels per axis, we need to use BMesh subdivision. */
  /* Convert mesh to BMesh. */
  BMeshCreateParams bmesh_create_params{};
  bmesh_create_params.use_toolflags = true; /* Required for BM_mesh_esubdivide */
  BMeshFromMeshParams bmesh_from_mesh_params{};
  bmesh_from_mesh_params.calc_face_normal = true;
  bmesh_from_mesh_params.calc_vert_normal = true;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &bmesh_create_params, &bmesh_from_mesh_params);
  if (!bm) {
    return nullptr;
  }

  /* Apply subdivision for each axis separately. */
  /* We subdivide edges that are primarily aligned with each axis. */
  const int levels[3] = {level_x, level_y, level_z};
  const float3 axis_dirs[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

  for (int axis = 0; axis < 3; axis++) {
    if (levels[axis] == 0) {
      continue;
    }

    /* Subdivide this axis the specified number of times. */
    for (int iteration = 0; iteration < levels[axis]; iteration++) {
      /* Clear all selections first. */
      BM_mesh_elem_hflag_disable_all(bm, BM_EDGE, BM_ELEM_SELECT, false);

      /* Select edges that are primarily aligned with this axis. */
      BMEdge *e;
      BMIter iter;
      BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
        float3 edge_dir;
        sub_v3_v3v3(edge_dir, e->v2->co, e->v1->co);
        float edge_len = len_v3(edge_dir);
        if (edge_len < 1e-6f) {
          continue;
        }
        mul_v3_fl(edge_dir, 1.0f / edge_len);

        /* Calculate alignment with this axis. */
        float alignment = fabsf(dot_v3v3(edge_dir, axis_dirs[axis]));

        /* Also check alignment with other axes to determine primary alignment. */
        float align_x = fabsf(edge_dir[0]);
        float align_y = fabsf(edge_dir[1]);
        float align_z = fabsf(edge_dir[2]);

        /* Determine which axis this edge is most aligned with. */
        int primary_axis = 0;
        if (align_x >= align_y && align_x >= align_z) {
          primary_axis = 0;
        }
        else if (align_y >= align_z) {
          primary_axis = 1;
        }
        else {
          primary_axis = 2;
        }

        /* Only subdivide edges that are primarily aligned with current axis. */
        if (primary_axis == axis && alignment > 0.5f) {
          BM_elem_select_set(bm, (BMElem *)e, true);
        }
      }

      /* Check if any edges are selected. */
      bool has_selected = false;
      BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
        if (BM_elem_flag_test(e, BM_ELEM_SELECT)) {
          has_selected = true;
          break;
        }
      }

      if (!has_selected) {
        /* No edges to subdivide in this iteration. */
        continue;
      }

      /* Perform subdivision on selected edges. */
      BM_mesh_esubdivide(bm,
                         BM_ELEM_SELECT,
                         0.0f,  /* smooth */
                         SUBD_FALLOFF_LIN,
                         false, /* use_smooth_even */
                         0.0f,  /* fractal */
                         0.0f,  /* along_normal */
                         1,     /* numcuts - one cut per iteration */
                         SUBDIV_SELECT_ORIG,
                         SUBD_CORNER_STRAIGHT_CUT,
                         false, /* use_single_edge */
                         true,  /* use_grid_fill */
                         false, /* use_only_quads */
                         0);    /* seed */
    }
  }

  /* Convert BMesh back to Mesh. */
  BMeshToMeshParams bmesh_to_mesh_params{};
  bmesh_to_mesh_params.calc_object_remap = false;
  Mesh *result = BKE_mesh_from_bmesh_nomain(bm, &bmesh_to_mesh_params, &mesh);

  BM_mesh_free(bm);

  geometry::debug_randomize_mesh_order(result);
  return result;
}
#endif /* WITH_OPENSUBDIV */

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
#ifdef WITH_OPENSUBDIV
  const int level_x = std::max(params.extract_input<int>("Level X"), 0);
  const int level_y = std::max(params.extract_input<int>("Level Y"), 0);
  const int level_z = std::max(params.extract_input<int>("Level Z"), 0);

  if (level_x == 0 && level_y == 0 && level_z == 0) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  /* Check if any level is too large. */
  if (level_x >= 16 || level_y >= 16 || level_z >= 16) {
    params.error_message_add(NodeWarningType::Error,
                             TIP_("The subdivision level is too large"));
    params.set_default_remaining_outputs();
    return;
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      Mesh *result = axis_subdivide_mesh(*mesh, level_x, level_y, level_z);
      if (result) {
        geometry_set.replace_mesh(result);
      }
    }
  });
#else
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenSubdiv"));
#endif
  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeSubdivideMeshAxis");
  ntype.ui_name = "Subdivide Mesh Axis";
  ntype.ui_description =
      "Divide mesh faces into smaller ones with different subdivision levels along each axis";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_subdivide_axis_cc
