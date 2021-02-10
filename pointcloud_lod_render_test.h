#pragma once

#include <cgv/base/node.h>
#include <cgv/math/fvec.h>
#include <cgv/media/color.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/data/data_view.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/texture.h>
#include <cgv/render/render_types.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/line_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv_gl/surfel_renderer.h>
#include <cgv_gl/clod_point_renderer.h>

#include <point_cloud.h>

#include <string>
#include <mutex>
#include <future>

#include "lib_begin.h"

class pointcloud_lod_render_test :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
public:
	pointcloud_lod_render_test();

	/// overload to return the type name of this object. By default the type interface is queried over get_type.
	std::string get_type_name() const { return "pointcloud_lod_render_test"; }

	bool self_reflect(cgv::reflect::reflection_handler& rh);

	void on_set(void* member_ptr);
	
	void on_register();
	
	void unregister();

	/// adjust view
	bool init(cgv::render::context& ctx);
	/// overload to draw the content of this drawable
	void draw(cgv::render::context& ctx);
	///
	void find_pointcloud(cgv::render::context& ctx);
	///
	void clear(cgv::render::context& ctx);
	/// 
	bool handle(cgv::gui::event& e);
	/// 
	void stream_help(std::ostream& os);
	///
	void create_gui();

protected:
	void timer_event(double t, double dt);

	void on_load_point_cloud_cb();
	void on_clear_point_cloud_cb();
	void on_randomize_position_cb();
	void on_reg_find_point_cloud_cb();
	void on_point_cloud_style_cb();
	void on_lod_mode_change();
	void prepare_point_cloud(point_cloud& pc);
	
private:
	std::string ply_path;
	point_cloud source_pc, crs_srs_pc;
	cgv::render::point_render_style source_prs;
	cgv::render::surfel_render_style source_srs;
	cgv::render::line_render_style lrs;
	cgv::render::rounded_cone_render_style rcrs;

	float rot_intensity;
	float trans_intensity;
	bool view_find_point_cloud;
	bool renderer_out_of_date = true;
	int lod_mode = (int)cgv::render::clod_point_renderer::LoDMode::RANDOM_POISSON;
	cgv::render::clod_point_renderer cp_renderer;
	cgv::render::clod_point_render_style cp_style;
};

#include <cgv/config/lib_end.h>