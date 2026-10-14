// Probe Bambu Studio's InterlockingGenerator through the real PrintObject::slice() pipeline.
// Builds 2-part objects (PETG part on filament 1, TPU part on filament 2), slices with given
// interlocking settings and reports where the regions were altered.
#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Surface.hpp"
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include "libslic3r/Format/bbs_3mf.hpp"
#include <cstdio>
#include <string>
#include <vector>

using namespace Slic3r;

struct Box { double x0, y0, z0, sx, sy, sz; };

struct Case {
    std::string name;
    std::vector<Box> petg, tpu;
    // settings
    bool   on = true;
    double width = 0.8;
    double orient = 22.5;
    int    layers = 2;
    int    depth = 2;
    int    avoid = 2;
    double layer_h = 0.2;
    // probe: "vertical interface at x = ix" or "horizontal interface at z = iz"
    bool   vertical = true;
    double ix = 0, iz = 0;
};

static ExPolygons region_slices(const Layer *layer, int filament)
{
    ExPolygons out;
    for (const LayerRegion *r : layer->regions())
        if ((int)r->region().config().wall_filament.value == filament)
            append(out, to_expolygons(r->slices.surfaces));
    return union_ex(out);
}

static DynamicPrintConfig make_config(const Case &c)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_key_value("filament_diameter", new ConfigOptionFloats({1.75, 1.75}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({"#FFFFFF", "#000000"}));
    config.set_key_value("filament_type", new ConfigOptionStrings({"PETG", "TPU"}));
    for (const std::string &key : config.keys()) {
        ConfigOption *opt = config.option(key, false);
        if (opt && opt->is_vector() && key.rfind("filament_", 0) == 0) {
            auto *v = static_cast<ConfigOptionVectorBase*>(opt);
            if (v->size() == 1) v->resize(2, nullptr);
        }
    }
    config.set_key_value("filament_shrink", new ConfigOptionPercents({100., 100.}));
    config.set_key_value("layer_height", new ConfigOptionFloat(c.layer_h));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(c.layer_h));
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("interlocking_beam", new ConfigOptionBool(c.on));
    config.set_key_value("interlocking_beam_width", new ConfigOptionFloat(c.width));
    config.set_key_value("interlocking_orientation", new ConfigOptionFloat(c.orient));
    config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(c.layers));
    config.set_key_value("interlocking_depth", new ConfigOptionInt(c.depth));
    config.set_key_value("interlocking_boundary_avoidance", new ConfigOptionInt(c.avoid));
    return config;
}

static void run(const Case &c)
{
    Model model;
    ModelObject *obj = model.add_object();
    obj->name = c.name;
    auto add = [&](const Box &b, int filament) {
        TriangleMesh m = make_cube(b.sx, b.sy, b.sz);
        m.translate(float(b.x0), float(b.y0), float(b.z0));
        ModelVolume *v = obj->add_volume(std::move(m));
        v->config.set_key_value("extruder", new ConfigOptionInt(filament));
    };
    for (const Box &b : c.petg) add(b, 1);
    for (const Box &b : c.tpu) add(b, 2);
    obj->add_instance();
    // keep model coordinates: place instance at origin, no centering
    obj->instances.front()->set_offset(Vec3d(0, 0, 0));
    obj->config.set_key_value("extruder", new ConfigOptionInt(1));

    Print print;
    DynamicPrintConfig config = make_config(c);
    print.apply(model, config);
    print.set_status_silent();
    print.set_no_check_flag(true);
    PrintObject *po = const_cast<PrintObject*>(print.objects().front());
    po->slice();

    // Object slices are in the object's "centered" frame. Find offset: model bbox center vs slice bbox.
    BoundingBoxf3 mbb = obj->raw_bounding_box();
    // compute slice-frame -> model-frame shift from the union of all layer slices' bbox
    BoundingBox sbb;
    for (const Layer *l : po->layers())
        for (const ExPolygon &e : l->lslices) sbb.merge(get_extents(e));
    const double shift_x = mbb.min.x() - unscale<double>(sbb.min.x());
    const double shift_y = mbb.min.y() - unscale<double>(sbb.min.y());

    printf("\n=== %s ===\n", c.name.c_str());
    if (getenv("ILK_DEBUG")) {
        printf("model bbox min (%.3f,%.3f,%.3f) max (%.3f,%.3f,%.3f); slice bbox x %.3f..%.3f y %.3f..%.3f; shift %.3f %.3f\n",
            mbb.min.x(), mbb.min.y(), mbb.min.z(), mbb.max.x(), mbb.max.y(), mbb.max.z(),
            unscale<double>(sbb.min.x()), unscale<double>(sbb.max.x()), unscale<double>(sbb.min.y()), unscale<double>(sbb.max.y()), shift_x, shift_y);
        int li = atoi(getenv("ILK_DEBUG"));
        for (const LayerRegion *r : po->layers()[li]->regions())
            printf("  region: wall_filament=%d sparse_infill_filament=%d slices=%zu\n", (int)r->region().config().wall_filament.value, (int)r->region().config().sparse_infill_filament.value, r->slices.surfaces.size());
        for (const LayerRegion *r : po->layers()[li]->regions()) for (const Surface &sf : r->slices.surfaces) { BoundingBox b = get_extents(sf.expolygon); printf("     raw surface area %.3f ccw=%d bbox x %.3f..%.3f\n", sf.expolygon.area()*SCALING_FACTOR*SCALING_FACTOR, (int)sf.expolygon.contour.is_counter_clockwise(), unscale<double>(b.min.x())+shift_x, unscale<double>(b.max.x())+shift_x); }
        if (li < (int)po->layers().size()) {
            const Layer *l = po->layers()[li];
            for (int f : {1, 2}) {
                ExPolygons e = region_slices(l, f);
                printf("  layer %d filament %d: %zu expolys\n", li, f, e.size());
                for (auto &x : e) { BoundingBox b = get_extents(x); printf("     bbox x %.3f..%.3f y %.3f..%.3f area %.3f holes %zu pts %zu\n",
                    unscale<double>(b.min.x())+shift_x, unscale<double>(b.max.x())+shift_x, unscale<double>(b.min.y())+shift_y, unscale<double>(b.max.y())+shift_y, x.area()*SCALING_FACTOR*SCALING_FACTOR, x.holes.size(), x.contour.points.size()); }
            }
        }
    }
    printf("settings: on=%d width=%.2f orient=%.1f layers=%d depth=%d avoid=%d layer_h=%.2f | regions=%zu layers=%zu\n",
           (int)c.on, c.width, c.orient, c.layers, c.depth, c.avoid, c.layer_h, po->num_printing_regions(), po->layers().size());

    int layers_with = 0;
    double total_intr_area = 0;
    double min_y = 1e9, max_y = -1e9, max_pen_petg_side = 0, max_pen_tpu_side = 0;
    int zmin_layer = 1 << 30, zmax_layer = -1;
    double min_x = 1e9, max_x = -1e9;
    std::string per_layer;
    for (const Layer *l : po->layers()) {
        ExPolygons pe = region_slices(l, 1), tp = region_slices(l, 2);
        // nominal half-spaces in model frame
        ExPolygons tpu_in_petg, petg_in_tpu;
        if (c.vertical) {
            const double ix_s = c.ix - shift_x; // interface x in slice frame
            Polygon left  = Polygon({ Point(scaled(-1000.), scaled(-1000.)), Point(scaled(ix_s), scaled(-1000.)), Point(scaled(ix_s), scaled(1000.)), Point(scaled(-1000.), scaled(1000.)) });
            Polygon right = Polygon({ Point(scaled(ix_s), scaled(-1000.)), Point(scaled(1000.), scaled(-1000.)), Point(scaled(1000.), scaled(1000.)), Point(scaled(ix_s), scaled(1000.)) });
            tpu_in_petg = intersection_ex(tp, Polygons{left});
            petg_in_tpu = intersection_ex(pe, Polygons{right});
            // drop slivers < 0.01 mm2 (numerical)
            auto clean = [](ExPolygons &e){ e.erase(std::remove_if(e.begin(), e.end(), [](const ExPolygon &p){ return std::abs(p.area()) < scaled(1.)*scaled(1.)*0.01; }), e.end()); };
            clean(tpu_in_petg); clean(petg_in_tpu);
            double a = 0; for (auto &e : tpu_in_petg) a += e.area(); for (auto &e : petg_in_tpu) a += e.area();
            a *= SCALING_FACTOR * SCALING_FACTOR;
            if (a > 0.01) {
                ++layers_with; total_intr_area += a;
                zmin_layer = std::min<int>(zmin_layer, l->id()); zmax_layer = std::max<int>(zmax_layer, l->id());
                for (auto &e : tpu_in_petg) { BoundingBox b = get_extents(e); min_y = std::min(min_y, unscale<double>(b.min.y()) + shift_y); max_y = std::max(max_y, unscale<double>(b.max.y()) + shift_y); max_pen_petg_side = std::max(max_pen_petg_side, ix_s - unscale<double>(b.min.x())); min_x = std::min(min_x, unscale<double>(b.min.x()) + shift_x);}
                for (auto &e : petg_in_tpu) { BoundingBox b = get_extents(e); min_y = std::min(min_y, unscale<double>(b.min.y()) + shift_y); max_y = std::max(max_y, unscale<double>(b.max.y()) + shift_y); max_pen_tpu_side = std::max(max_pen_tpu_side, unscale<double>(b.max.x()) - ix_s); max_x = std::max(max_x, unscale<double>(b.max.x()) + shift_x);}
                char buf[256];
                snprintf(buf, sizeof(buf), "  L%-3zu z=%.2f  TPU-in-PETG islands=%zu area=%.2f | PETG-in-TPU islands=%zu area=%.2f\n",
                         l->id(), l->print_z, tpu_in_petg.size(), [&]{double s=0; for(auto&e:tpu_in_petg) s+=e.area(); return s*SCALING_FACTOR*SCALING_FACTOR;}(),
                         petg_in_tpu.size(), [&]{double s=0; for(auto&e:petg_in_tpu) s+=e.area(); return s*SCALING_FACTOR*SCALING_FACTOR;}());
                per_layer += buf;
            }
        } else {
            // horizontal interface at z = iz: PETG below, TPU above. Count TPU present below iz and PETG above iz.
            double a = 0; size_t n = 0;
            const ExPolygons &intr = (l->print_z <= c.iz + 1e-6) ? tp : pe;
            for (auto &e : intr) a += e.area();
            a *= SCALING_FACTOR * SCALING_FACTOR;
            n = intr.size();
            if (a > 0.01) {
                ++layers_with; total_intr_area += a;
                zmin_layer = std::min<int>(zmin_layer, l->id()); zmax_layer = std::max<int>(zmax_layer, l->id());
                for (auto &e : intr) { BoundingBox b = get_extents(e);
                    min_y = std::min(min_y, unscale<double>(b.min.y()) + shift_y); max_y = std::max(max_y, unscale<double>(b.max.y()) + shift_y);
                    min_x = std::min(min_x, unscale<double>(b.min.x()) + shift_x); max_x = std::max(max_x, unscale<double>(b.max.x()) + shift_x); }
                char buf[200];
                snprintf(buf, sizeof(buf), "  L%-3zu z=%.2f  %s islands=%zu area=%.2f\n", l->id(), l->print_z,
                         (l->print_z <= c.iz + 1e-6) ? "TPU-below-interface" : "PETG-above-interface", n, a);
                per_layer += buf;
            }
        }
    }
    if (layers_with == 0) {
        printf("RESULT: NO interlocking geometry generated.\n");
        return;
    }
    const Layer *l0 = po->layers()[zmin_layer], *l1 = po->layers()[zmax_layer];
    printf("RESULT: interlocking on %d layers (layer %d..%d, print_z %.2f..%.2f), intruding area total %.1f mm2\n",
           layers_with, zmin_layer, zmax_layer, l0->print_z, l1->print_z, total_intr_area);
    if (c.vertical)
        printf("        extent along interface y: %.2f..%.2f ; TPU reaches into PETG by %.2f mm (to x=%.2f), PETG into TPU by %.2f mm (to x=%.2f)\n",
               min_y, max_y, max_pen_petg_side, min_x, max_pen_tpu_side, max_x);
    else
        printf("        xy extent of intruding material: x %.2f..%.2f, y %.2f..%.2f\n", min_x, max_x, min_y, max_y);
    if (getenv("ILK_VERBOSE")) fputs(per_layer.c_str(), stdout);
    if (const char *dump = getenv("ILK_DUMP")) {
        // dump all layers' region polygons (model frame, mm) as simple text: L <layer> <z> F <filament> P <npts> x y x y ...
        std::string fn = std::string(dump);
        FILE *f = fopen(fn.c_str(), "w");
        for (const Layer *l : po->layers())
            for (int fil : {1, 2})
                for (const ExPolygon &e : region_slices(l, fil))
                    for (const Polygon &pg : to_polygons(e)) {
                        fprintf(f, "L %zu %.3f F %d %d P %zu", l->id(), l->print_z, fil, (int)pg.is_counter_clockwise(), pg.points.size());
                        for (const Point &pt : pg.points) fprintf(f, " %.4f %.4f", unscale<double>(pt.x()) + shift_x, unscale<double>(pt.y()) + shift_y);
                        fprintf(f, "\n");
                    }
        fclose(f);
    }
}


// Slice one model object (copied into a fresh model) and return per-layer region slices for filament 1 and 2.
static std::vector<std::pair<ExPolygons, ExPolygons>> slice_object(const ModelObject &src, const DynamicPrintConfig &config, int force_beam /*-1 keep*/, int force_avoid)
{
    Model model;
    ModelObject *obj = model.add_object(src);
    if (force_beam >= 0) obj->config.set_key_value("interlocking_beam", new ConfigOptionBool(force_beam != 0));
    if (force_avoid >= 0) obj->config.set_key_value("interlocking_boundary_avoidance", new ConfigOptionInt(force_avoid));
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.set_no_check_flag(true);
    PrintObject *po = const_cast<PrintObject*>(print.objects().front());
    po->slice();
    std::vector<std::pair<ExPolygons, ExPolygons>> out;
    for (const Layer *l : po->layers()) out.emplace_back(region_slices(l, 1), region_slices(l, 2));
    return out;
}

static int run_3mf(int argc, char **argv)
{
    const std::string path = argv[2];
    DynamicPrintConfig loaded;
    ConfigSubstitutionContext subs{ForwardCompatibilitySubstitutionRule::Enable};
    Model model = Model::read_from_file(path, &loaded, &subs, LoadStrategy(int(LoadStrategy::LoadModel) | int(LoadStrategy::LoadConfig) | int(LoadStrategy::AddDefaultInstances)));
    Case dflt; DynamicPrintConfig config = make_config(dflt);
    // object-level keys from the 3MF override; global interlocking off unless object says so
    config.set_key_value("interlocking_beam", new ConfigOptionBool(false));
    int force_avoid = getenv("ILK_AVOID") ? atoi(getenv("ILK_AVOID")) : -1;
    int force_beam = getenv("ILK_BEAM") ? atoi(getenv("ILK_BEAM")) : -1;
    for (const ModelObject *mo : model.objects) {
        bool match = argc <= 3;
        for (int i = 3; i < argc; ++i) if (mo->name.rfind(argv[i], 0) == 0) match = true;
        if (!match) continue;
        std::string keys; for (auto &k : mo->config.keys()) keys += k + "=" + mo->config.opt_serialize(k) + " ";
        auto on  = slice_object(*mo, config, force_beam, force_avoid);
        auto off = slice_object(*mo, config, 0, -1);
        int nl = 0; double tot = 0; int first = -1, last = -1;
        for (size_t i = 0; i < on.size() && i < off.size(); ++i) {
            double a = 0;
            for (auto &e : diff_ex(on[i].first, off[i].first)) a += e.area();
            for (auto &e : diff_ex(on[i].second, off[i].second)) a += e.area();
            a *= SCALING_FACTOR * SCALING_FACTOR;
            if (a > 0.01) { ++nl; tot += a; if (first < 0) first = (int)i; last = (int)i; }
        }
        printf("%-48s layers=%zu  keys: %s\n   -> interlocking changed geometry on %d layers (%d..%d), swapped area %.1f mm2\n",
               mo->name.c_str(), on.size(), keys.c_str(), nl, first, last, tot);
        // radial reach of the woven zone, measured from the bore centre (centre of the TPU's
        // bounding box in each layer without interlocking)
        double petg_rmin = 1e9, petg_rmax = 0, tpu_rmin = 1e9, tpu_rmax = 0;
        for (size_t i = 0; i < on.size() && i < off.size(); ++i) {
            if (off[i].second.empty()) continue;
            BoundingBox bb = get_extents(off[i].second);
            double cx = unscale<double>(bb.min.x() + bb.max.x()) / 2, cy = unscale<double>(bb.min.y() + bb.max.y()) / 2;
            cx = unscale<double>(bb.min.x()) + (unscale<double>(bb.max.y()) - unscale<double>(bb.min.y())) / 2;  // bore centre: liner reaches -x fully
            auto reach = [&](const ExPolygons &ex, double &rmin, double &rmax) {
                for (const ExPolygon &e : ex) {
                    if (e.area() * SCALING_FACTOR * SCALING_FACTOR < 0.02) continue;
                    for (const Point &p : e.contour.points) {
                        double r = std::hypot(unscale<double>(p.x()) - cx, unscale<double>(p.y()) - cy);
                        rmin = std::min(rmin, r); rmax = std::max(rmax, r);
                    }
                }
            };
            reach(diff_ex(on[i].first, off[i].first), petg_rmin, petg_rmax);
            reach(diff_ex(on[i].second, off[i].second), tpu_rmin, tpu_rmax);
        }
        printf("   -> PETG beams placed where TPU was: radius %.2f .. %.2f mm from the bore centre\n", petg_rmin, petg_rmax);
        // where are the deep PETG beams? angle histogram of PETG-gained points below ILK_RDEEP
        if (getenv("ILK_RDEEP")) {
            double rdeep = atof(getenv("ILK_RDEEP"));
            std::vector<int> hist(36, 0);
            int layers_with_deep = 0;
            for (size_t i = 0; i < on.size() && i < off.size(); ++i) {
                if (off[i].second.empty()) continue;
                BoundingBox bb = get_extents(off[i].second);
                double cy = unscale<double>(bb.min.y() + bb.max.y()) / 2;
                double cx = unscale<double>(bb.min.x()) + (unscale<double>(bb.max.y()) - unscale<double>(bb.min.y())) / 2;
                bool deep = false;
                for (const ExPolygon &e : diff_ex(on[i].first, off[i].first)) {
                    if (e.area() * SCALING_FACTOR * SCALING_FACTOR < 0.02) continue;
                    for (const Point &p : e.contour.points) {
                        double x = unscale<double>(p.x()) - cx, y = unscale<double>(p.y()) - cy;
                        if (std::hypot(x, y) < rdeep) {
                            double a = std::atan2(y, x) * 180 / M_PI; if (a < 0) a += 360;
                            hist[std::min(35, (int)(a / 10))]++; deep = true;
                        }
                    }
                }
                if (deep) ++layers_with_deep;
            }
            printf("   -> PETG beam points below r=%.2f on %d layers, by 10-deg bin (0 = slit):", rdeep, layers_with_deep);
            for (int b = 0; b < 36; ++b) if (hist[b]) printf(" %d:%d", b * 10, hist[b]);
            printf("\n");
        }
        printf("   -> TPU beams placed where PETG was: radius %.2f .. %.2f mm\n", tpu_rmin, tpu_rmax);
    }
    return 0;
}

int main(int argc, char **argv)
{
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::fatal);
    std::vector<Case> cases;
    if (argc > 2 && std::string(argv[1]) == "--3mf") return run_3mf(argc, argv);
    const std::string which = argc > 1 ? argv[1] : "all";

    // A: two 20x20x10 blocks side by side, shared face at x=20
    auto blocks = [](const std::string &n) { Case c; c.name = n; c.petg = {{0,0,0,20,20,10}}; c.tpu = {{20,0,0,20,20,10}}; c.ix = 20; return c; };
    { Case c = blocks("A0 blocks 20x20x10, interlocking OFF"); c.on = false; cases.push_back(c); }
    { Case c = blocks("A1 blocks, defaults (w0.8 o22.5 L2 d2 a2)"); cases.push_back(c); }
    { Case c = blocks("A2 blocks, avoid=1"); c.avoid = 1; cases.push_back(c); }
    { Case c = blocks("A3 blocks, avoid=0"); c.avoid = 0; cases.push_back(c); }
    { Case c = blocks("A4 blocks, depth=3"); c.depth = 3; cases.push_back(c); }
    { Case c = blocks("A5 blocks, depth=4"); c.depth = 4; cases.push_back(c); }
    { Case c = blocks("A6 blocks, depth=6"); c.depth = 6; cases.push_back(c); }
    { Case c = blocks("A7 blocks, width=1.2"); c.width = 1.2; cases.push_back(c); }
    { Case c = blocks("A8 blocks, width=0.4"); c.width = 0.4; cases.push_back(c); }
    { Case c = blocks("A9 blocks, orient=0"); c.orient = 0; cases.push_back(c); }
    { Case c = blocks("A10 blocks, orient=45"); c.orient = 45; cases.push_back(c); }
    { Case c = blocks("A11 blocks, beam layers=1"); c.layers = 1; cases.push_back(c); }
    { Case c = blocks("A12 blocks, beam layers=3"); c.layers = 3; cases.push_back(c); }
    // B: gap tests (TPU block shifted by a small gap)
    for (double g : {0.0005, 0.002, 0.01, 0.05}) {
        Case c = blocks("B gap " + std::to_string(g) + " mm between blocks, defaults"); c.tpu = {{20 + g,0,0,20,20,10}}; cases.push_back(c);
    }
    { Case c = blocks("B gap 0.05 mm, avoid=0"); c.tpu = {{20.05,0,0,20,20,10}}; c.avoid = 0; cases.push_back(c); }
    { Case c = blocks("B gap 0.01 mm, avoid=1"); c.tpu = {{20.01,0,0,20,20,10}}; c.avoid = 1; cases.push_back(c); }
    // B overlap: TPU overlaps PETG by 0.3 mm (TPU added last -> carves PETG)
    { Case c = blocks("B overlap 0.3 mm (TPU later carves PETG), defaults"); c.tpu = {{19.7,0,0,20.3,20,10}}; c.ix = 19.7; cases.push_back(c); }
    // C: thin coupons like the bond test: 12 wide x 3.2 tall, joined end to end
    auto coupon = [](const std::string &n, double t, double w) { Case c; c.name = n; c.petg = {{0,0,0,20,w,t}}; c.tpu = {{20,0,0,20,w,t}}; c.ix = 20; return c; };
    { Case c = coupon("C1 coupon 12w x 3.2t, defaults", 3.2, 12); cases.push_back(c); }
    { Case c = coupon("C2 coupon 12w x 3.2t, avoid=1", 3.2, 12); c.avoid = 1; cases.push_back(c); }
    { Case c = coupon("C3 coupon 12w x 3.2t, avoid=0", 3.2, 12); c.avoid = 0; cases.push_back(c); }
    { Case c = coupon("C4 coupon 12w x 4.8t, defaults", 4.8, 12); cases.push_back(c); }
    { Case c = coupon("C5 coupon 12w x 6.4t, defaults", 6.4, 12); cases.push_back(c); }
    { Case c = coupon("C6 coupon 8w x 10t, defaults", 10, 8); cases.push_back(c); }
    { Case c = coupon("C7 coupon 6w x 10t, defaults", 10, 6); cases.push_back(c); }
    { Case c = coupon("C8 coupon 6w x 10t, avoid=1", 10, 6); c.avoid = 1; cases.push_back(c); }
    for (double t : {1.6, 2.0, 2.4}) { Case c = coupon("E thickness " + std::to_string(t) + " x 12w, defaults", t, 12); cases.push_back(c); }
    for (double w : {2.0, 3.0, 4.0}) { Case c = coupon("E width " + std::to_string(w) + " x 10t, defaults", 10, w); cases.push_back(c); }
    for (double w : {2.0, 3.0}) { Case c = coupon("E width " + std::to_string(w) + " x 10t, avoid=1", 10, w); c.avoid = 1; cases.push_back(c); }
    for (double w : {5.0}) { Case c = coupon("F width " + std::to_string(w) + " x 10t, defaults", 10, w); cases.push_back(c); }
    for (double w : {2.0, 1.2}) { Case c = coupon("F width " + std::to_string(w) + " x 10t, avoid=0", 10, w); c.avoid = 0; cases.push_back(c); }
    { Case c = coupon("F thickness 1.6 x 12w, avoid=1", 1.6, 12); c.avoid = 1; cases.push_back(c); }
    { Case c = coupon("F thickness 0.8 x 12w, avoid=0", 0.8, 12); c.avoid = 0; cases.push_back(c); }
    // D: horizontal interface: TPU 20x20x5 on top of PETG 20x20x5
    { Case c; c.name = "D1 TPU on top of PETG (horizontal interface z=5), defaults"; c.petg = {{0,0,0,20,20,5}}; c.tpu = {{0,0,5,20,20,5}}; c.vertical = false; c.iz = 5; cases.push_back(c); }
    { Case c; c.name = "D2 horizontal interface, depth=4"; c.petg = {{0,0,0,20,20,5}}; c.tpu = {{0,0,5,20,20,5}}; c.vertical = false; c.iz = 5; c.depth = 4; cases.push_back(c); }
    { Case c; c.name = "D3 horizontal, TPU pad 2.0 mm thick on PETG, defaults"; c.petg = {{0,0,0,20,20,5}}; c.tpu = {{0,0,5,20,20,2}}; c.vertical = false; c.iz = 5; cases.push_back(c); }
    { Case c; c.name = "D4 horizontal, TPU pad 2.0 mm thick on PETG, avoid=1"; c.petg = {{0,0,0,20,20,5}}; c.tpu = {{0,0,5,20,20,2}}; c.vertical = false; c.iz = 5; c.avoid = 1; cases.push_back(c); }

    for (const Case &c : cases) {
        if (which != "all" && c.name.rfind(which, 0) != 0) continue;
        try { run(c); } catch (const std::exception &e) { printf("\n=== %s ===\nEXCEPTION: %s\n", c.name.c_str(), e.what()); }
        fflush(stdout);
    }
    return 0;
}
