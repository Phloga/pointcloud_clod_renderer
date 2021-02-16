#include <cgv/base/base.h>
#include "pointcloud_lod_render_test.h"
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/file_dialog.h>
#include <random>
#include <chrono>
#include <numeric>

using namespace std;
using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::type;
using namespace cgv::gui;
using namespace cgv::data;
using namespace cgv::utils;
using namespace cgv::render;

pointcloud_lod_render_test::pointcloud_lod_render_test() {
	set_name("pointcloud_lod_render_test");

	source_prs.measure_point_size_in_pixel = false;
	source_prs.point_size = 1.00f;
	source_prs.blend_width_in_pixel = 1.0f;
	source_prs.blend_points = true;

	source_srs.measure_point_size_in_pixel = false;
	source_srs.point_size = 1.00f;
	source_srs.blend_width_in_pixel = 1.0f;
	source_srs.blend_points = true;
	source_srs.illumination_mode = cgv::render::IM_TWO_SIDED;

	rot_intensity = 0.2f;
	trans_intensity = 0.1;
	lrs.line_width = 1.0f;
	rcrs.radius = 0.001f;
}

bool pointcloud_lod_render_test::self_reflect(cgv::reflect::reflection_handler & rh)
{
	return	rh.reflect_member("ply_path", ply_path);
}

void pointcloud_lod_render_test::on_set(void * member_ptr)
{
}

void pointcloud_lod_render_test::on_register()
{
}

void pointcloud_lod_render_test::unregister()
{
}

bool pointcloud_lod_render_test::init(cgv::render::context & ctx)
{
	cgv::gui::connect_vr_server(true);

	ctx.set_bg_clr_idx(3);
	ctx.set_bg_color(0, 0, 0, 0.9);
	cgv::render::view* view_ptr = find_view_as_node();

	if (view_ptr) {
		view_ptr->set_view_up_dir(vec3(0, -1, 0));
		view_ptr->set_focus(vec3(0, 0, 0));
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_interactor* vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	//cgv::render::ref_point_renderer(ctx, 1);
	cgv::render::ref_surfel_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	cp_renderer.init(ctx);
	ctx.set_bg_color(0.7, 0.7, 0.8, 1.0);
	return true;
}


void pointcloud_lod_render_test::draw(cgv::render::context & ctx)
{
	//ctx.push_modelview_matrix();
	cp_renderer.set_render_style(cp_style);

	if (source_pc.get_nr_points() > 0) {
		if (renderer_out_of_date) {
			rgb color(1.0, 0.0, 0.0);
			//find weighted center
			vec3 centroid(0.f), pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
			float scale = 1.f;
			for (int i = 0; i < source_pc.get_nr_points(); ++i) {
				centroid += source_pc.pnt(i);
				pmin.x() = std::min(source_pc.pnt(i).x(), pmin.x()); pmin.y() = std::min(source_pc.pnt(i).y(), pmin.y());
				pmin.z() = std::min(source_pc.pnt(i).z(),pmin.z());
				pmax.x() = std::max(source_pc.pnt(i).x(), pmax.x()); pmax.y() = std::max(source_pc.pnt(i).y(), pmax.y());
				pmax.z() = std::max(source_pc.pnt(i).z(), pmax.z());
			}
			centroid /= source_pc.get_nr_points();
			vec3 ext = (pmax - pmin);
			scale = (1.0 / static_cast<double>(*std::max_element(ext.begin(),ext.end())));

			vector<point_cloud::Pnt> P(source_pc.get_nr_points());
			vector<point_cloud::Clr> C(source_pc.get_nr_points());

			for (int i = 0; i < source_pc.get_nr_points(); ++i) {
				P[i] = (source_pc.pnt(i) - centroid)*scale;
				if (source_pc.has_colors()) {
					C[i] = source_pc.clr(i);
				}
				else {
					C[i] = rgb8(color);
				}
			}
			//vector<cgv::render::render_types::rgba> colors(source_pc.get_nr_points(), rgba(color.x(), color.y(), color.z(), 0.f));

			cp_renderer.set_positions(ctx, P);
			cp_renderer.set_colors(ctx, C);
			cp_renderer.generate_lods((cgv::render::clod_point_renderer::LoDMode)lod_mode);
			renderer_out_of_date = false;
		}
		if (cp_renderer.enable(ctx))
			cp_renderer.draw(ctx, 0, source_pc.get_nr_points());
	}
	//ctx.pop_modelview_matrix();

	if (view_find_point_cloud) {
		find_pointcloud(ctx);
		view_find_point_cloud = false;
	}
}

void pointcloud_lod_render_test::find_pointcloud(cgv::render::context & ctx)
{
	cgv::render::view* view_ptr = find_view_as_node();
	if (view_ptr) {
		const point_cloud_types::Box& sb = source_pc.box();
		point_cloud_types::Box aabb(sb);

		view_ptr->set_focus(aabb.get_center());
		
		view_ptr->move(view_ptr->get_depth_of_focus()-1.0);
		
	}

	
}

void pointcloud_lod_render_test::clear(cgv::render::context & ctx)
{
	//cgv::render::ref_point_renderer(ctx, -1);
	cgv::render::ref_surfel_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
}

bool pointcloud_lod_render_test::handle(cgv::gui::event & e)
{
	return false;
}

void pointcloud_lod_render_test::stream_help(std::ostream & os)
{
}

void pointcloud_lod_render_test::create_gui()
{
	add_decorator("Point cloud", "heading", "level=1");
	connect_copy(add_button("load point cloud")->click, rebind(this, &pointcloud_lod_render_test::on_load_point_cloud_cb));
	connect_copy(add_button("clear point cloud")->click, rebind(this, &pointcloud_lod_render_test::on_clear_point_cloud_cb));
	connect_copy(add_button("randomize position")->click, rebind(this, &pointcloud_lod_render_test::on_randomize_position_cb));
	add_member_control(this, "rotation intensity", rot_intensity, "value_slider", "min=0.01;max=1.0;log=false;ticks=true");
	add_member_control(this,"translation intensity", trans_intensity, "value_slider", "min=0.01;max=1.0;log=false;ticks=true");
	connect_copy(add_button("find point cloud")->click, rebind(this, &pointcloud_lod_render_test::on_reg_find_point_cloud_cb));
	std::string mode_defs = "enums='random=2;potree=1'";
	connect_copy(add_control("lod generator", (DummyEnum&)lod_mode, "dropdown", mode_defs)->value_change, rebind(this, &pointcloud_lod_render_test::on_lod_mode_change));


	add_decorator("point cloud", "heading", "level=2");

	if (begin_tree_node("clod_render_style", cp_style, false)) {
		align("\a");
		add_gui("clod style", cp_style);
		align("\b");
		end_tree_node(cp_style);
	}
}

void pointcloud_lod_render_test::timer_event(double t, double dt)
{
	//post_redraw();
}

void pointcloud_lod_render_test::on_load_point_cloud_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud", "Point cloud files (obj,ply):*.obj;*.ply");
	source_pc.read(fn);
	renderer_out_of_date = true;
	post_redraw();
}


void pointcloud_lod_render_test::on_clear_point_cloud_cb()
{
	source_pc.clear();
	crs_srs_pc.clear();
	renderer_out_of_date = true;
	post_redraw();
}

void pointcloud_lod_render_test::on_randomize_position_cb()
{
	uniform_real_distribution<float> angle_distribution(0.f, 3.142f);
	uniform_real_distribution<float> direction_distribution(0.f, 0.05f);
	random_device rng;
	float angle = rot_intensity*angle_distribution(rng);
	source_pc.rotate(cgv::math::quaternion<float>(normalize(vec3(direction_distribution(rng), direction_distribution(rng), direction_distribution(rng))), angle));
	source_pc.translate(trans_intensity*vec3(direction_distribution(rng), direction_distribution(rng), direction_distribution(rng)));
	renderer_out_of_date = true;
	post_redraw();
}

void pointcloud_lod_render_test::on_reg_find_point_cloud_cb()
{
	view_find_point_cloud = true;
	post_redraw();
}

void pointcloud_lod_render_test::on_point_cloud_style_cb()
{
	post_redraw();
}

void pointcloud_lod_render_test::on_lod_mode_change()
{

}

#include "lib_begin.h"

extern CGV_API object_registration<pointcloud_lod_render_test> kc_or("");