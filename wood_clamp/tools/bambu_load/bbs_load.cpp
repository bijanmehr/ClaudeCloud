// bbs_load: load 3MF / STL files with Bambu Studio's own libslic3r, following the
// exact code paths of Plater::priv::load_files() (src/slic3r/GUI/Plater.cpp), and
// print what the GUI would end up with.
//
// Usage:  bbs_load file.3mf [more.3mf ...]
//         bbs_load a.stl [b.stl ...]
//         bbs_load --split one.stl          (only the "Split to parts" test)
//
// Env:    BBS_LOG=<0..5>    boost log level of libslic3r (default 1 = errors)
//         BBS_BED_X / BBS_BED_Y   bed centre used by the GUI placement code (default 175,160 = H2D 350x320)
//         BBS_FILAMENTS=<n>  number of filaments in the current project (max_extruders for
//                            convert_multipart_object / ModelVolume::split), default 2

#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
namespace fs = boost::filesystem;

static Vec2d g_bed_center(175., 160.);
static unsigned int g_filaments = 2;

static const char *vtype_name(ModelVolumeType t)
{
    switch (t) {
    case ModelVolumeType::MODEL_PART:         return "MODEL_PART";
    case ModelVolumeType::NEGATIVE_VOLUME:    return "NEGATIVE_VOLUME";
    case ModelVolumeType::PARAMETER_MODIFIER: return "PARAMETER_MODIFIER";
    case ModelVolumeType::SUPPORT_BLOCKER:    return "SUPPORT_BLOCKER";
    case ModelVolumeType::SUPPORT_ENFORCER:   return "SUPPORT_ENFORCER";
    default:                                  return "INVALID";
    }
}

static std::string v3(const Vec3d &v)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return ss.str();
}

static std::string bbox_str(const BoundingBoxf3 &bb)
{
    if (!bb.defined) return "<undefined>";
    return "min " + v3(bb.min) + " max " + v3(bb.max) + " size " + v3(bb.size());
}

static std::string opt_extruder(const ModelConfigObject &cfg)
{
    const ConfigOption *opt = cfg.option("extruder");
    if (opt == nullptr) return "<unset>";
    return std::to_string(opt->getInt());
}

static std::string cfg_keys(const ModelConfigObject &cfg)
{
    std::string out;
    for (const std::string &k : cfg.keys()) {
        if (!out.empty()) out += ",";
        out += k;
        if (k != "extruder") {
            const ConfigOption *o = cfg.option(k);
            if (o) out += "=" + o->serialize();
        }
    }
    return out.empty() ? "<none>" : out;
}

static std::string model_str(const Model &model, const std::string &indent = "  ")
{
    std::ostringstream out;
    out << indent << "objects: " << model.objects.size() << "\n";
    for (size_t oi = 0; oi < model.objects.size(); ++oi) {
        const ModelObject *o = model.objects[oi];
        out << indent << "object[" << oi << "] name=\"" << o->name << "\""
                  << "  object.config.extruder=" << opt_extruder(o->config)
                  << "  object.config keys: " << cfg_keys(o->config) << "\n";
        out << indent << "  origin_translation=" << v3(o->origin_translation)
                  << "  parts_count=" << o->parts_count() << "\n";
        out << indent << "  instances: " << o->instances.size() << "\n";
        for (size_t ii = 0; ii < o->instances.size(); ++ii) {
            const ModelInstance *inst = o->instances[ii];
            const Geometry::Transformation &t = inst->get_transformation();
            out << indent << "    instance[" << ii << "] offset=" << v3(inst->get_offset())
                      << " rotation=" << v3(t.get_rotation()) << " scale=" << v3(t.get_scaling_factor())
                      << " printable=" << inst->printable << "\n";
            out << indent << "      instance bbox (parts only, world): " << bbox_str(o->instance_bounding_box(ii)) << "\n";
        }
        out << indent << "  volumes: " << o->volumes.size() << "\n";
        for (size_t vi = 0; vi < o->volumes.size(); ++vi) {
            const ModelVolume *v = o->volumes[vi];
            const TriangleMesh &mesh = v->mesh();
            const TriangleMeshStats &st = mesh.stats();
            out << indent << "    volume[" << vi << "] name=\"" << v->name << "\"\n";
            out << indent << "      type=" << vtype_name(v->type())
                      << "  volume.config.extruder=" << opt_extruder(v->config)
                      << "  extruder_id()=" << v->extruder_id();
            std::vector<int> exts = v->get_extruders();
            out << "  get_extruders()=[";
            for (size_t k = 0; k < exts.size(); ++k) out << (k ? "," : "") << exts[k];
            out << "]\n";
            out << indent << "      volume.config keys: " << cfg_keys(v->config) << "\n";
            out << indent << "      triangles=" << mesh.its.indices.size() << " vertices=" << mesh.its.vertices.size()
                      << " shells(number_of_parts)=" << st.number_of_parts
                      << " its_number_of_patches=" << its_number_of_patches(mesh.its)
                      << " open_edges=" << st.open_edges
                      << " volume=" << std::fixed << std::setprecision(3) << its_volume(mesh.its)
                      << " is_splittable=" << v->is_splittable() << "\n";
            out << indent << "      volume offset=" << v3(v->get_offset()) << " rotation=" << v3(v->get_rotation())
                      << " scale=" << v3(v->get_scaling_factor()) << " mirror=" << v3(v->get_mirror()) << "\n";
            out << indent << "      mesh bbox (volume local): " << bbox_str(mesh.bounding_box()) << "\n";
            out << indent << "      bbox in object coords   : " << bbox_str(mesh.transformed_bounding_box(v->get_matrix())) << "\n";
            for (size_t ii = 0; ii < o->instances.size(); ++ii) {
                Transform3d world = o->instances[ii]->get_matrix() * v->get_matrix();
                out << indent << "      bbox WORLD (instance " << ii << "): " << bbox_str(mesh.transformed_bounding_box(world)) << "\n";
            }
            if (o->instances.empty())
                out << indent << "      (no instances -> no world bbox)\n";
        }
    }
    return out.str();
}

static void print_model(const Model &model, const std::string &indent = "  ") { std::cout << model_str(model, indent); }

static void print_config_summary(const DynamicPrintConfig &cfg)
{
    std::cout << "  loaded project config: " << (cfg.empty() ? "EMPTY" : std::to_string(cfg.keys().size()) + " keys") << "\n";
    if (cfg.empty()) return;
    const char *keys[] = { "printer_settings_id", "printer_model", "print_settings_id", "filament_settings_id", "filament_colour",
                           "filament_type", "nozzle_diameter", "inherits", "default_print_profile", "filament_map", "filament_map_mode" };
    for (const char *k : keys) {
        if (cfg.has(k)) {
            std::string s = cfg.option(k)->serialize();
            if (s.size() > 200) s = s.substr(0, 200) + "...";
            std::cout << "    " << k << " = " << s << "\n";
        }
    }
}

static std::string strategy_str(LoadStrategy s)
{
    std::string out;
    auto add = [&](LoadStrategy f, const char *n) { if (s & f) { if (!out.empty()) out += "|"; out += n; } };
    add(LoadStrategy::AddDefaultInstances, "AddDefaultInstances");
    add(LoadStrategy::CheckVersion, "CheckVersion");
    add(LoadStrategy::LoadModel, "LoadModel");
    add(LoadStrategy::LoadConfig, "LoadConfig");
    add(LoadStrategy::LoadAuxiliary, "LoadAuxiliary");
    add(LoadStrategy::Silence, "Silence");
    add(LoadStrategy::ImperialUnits, "ImperialUnits");
    return out + " (=" + std::to_string(static_cast<int>(s)) + ")";
}

// Plater::priv::load_model_objects() (Plater.cpp ~9940), geometry relevant part only.
// AUTOPLACEMENT_ON_LOAD is #defined right above it (Plater.cpp:9938), so objects without instances get
// center_around_origin() + a new instance, ensure_on_bed(), and afterwards the new instance is moved to the
// bed centre in XY (if the plate is empty; otherwise to canvas3D()->get_nearest_empty_cell(), not modelled here).
static void gui_load_model_objects(Model &target, const ModelObjectPtrs &objs, bool allow_negative_z)
{
    std::vector<ModelInstance *> new_instances;
    for (ModelObject *model_object : objs) {
        ModelObject *object = target.add_object(*model_object);
        object->sort_volumes(true);
        if (model_object->instances.empty()) {
            object->center_around_origin();
            new_instances.emplace_back(object->add_instance());
        }
        object->ensure_on_bed(allow_negative_z);
    }
    for (ModelInstance *instance : new_instances) {
        Vec3d offset = instance->get_offset();
        instance->set_offset(Vec3d(g_bed_center.x(), g_bed_center.y(), offset.z()));
    }
}

// ----------------------------------------------------------------------------------------------
// 3MF
// ----------------------------------------------------------------------------------------------
static void run_3mf(const std::string &path, const char *label, LoadStrategy entry_strategy)
{
    std::cout << "\n=== 3MF " << label << " ===\n";
    // Plater::priv::load_files(): (Plater.cpp ~8603-8608)
    LoadStrategy strategy = entry_strategy;
    bool load_model  = strategy & LoadStrategy::LoadModel;
    bool load_config = strategy & LoadStrategy::LoadConfig;
    if (load_model && load_config) strategy = strategy | LoadStrategy::LoadAuxiliary;
    if (load_config) strategy = strategy | LoadStrategy::CheckVersion;
    std::cout << "  entry strategy: " << strategy_str(entry_strategy) << "\n";
    std::cout << "  strategy passed to read_from_archive/load_bbs_3mf: " << strategy_str(strategy) << "\n";

    // (1) direct load_bbs_3mf() call, for diagnostics (return value, is_bbl_3mf, version)
    {
        DynamicPrintConfig        config;
        ConfigSubstitutionContext subs{ForwardCompatibilitySubstitutionRule::Enable};
        Model                     model;
        PlateDataPtrs             plate_data;
        std::vector<Preset *>     project_presets;
        bool                      is_bbl_3mf = false;
        Semver                    file_version;
        std::map<int, std::vector<std::string>> color_group_map;
        VolumeColorInfoMap        volume_color_data;
        try {
            bool ret = load_bbs_3mf(path.c_str(), &config, &subs, &model, &plate_data, &project_presets, &is_bbl_3mf, &file_version,
                                    nullptr, strategy, nullptr, 0, &color_group_map, &volume_color_data);
            std::cout << "  [direct load_bbs_3mf] return=" << (ret ? "true" : "false") << " is_bbl_3mf=" << is_bbl_3mf
                      << " file_version=" << file_version.to_string() << " objects=" << model.objects.size()
                      << " plates=" << plate_data.size() << " project_presets=" << project_presets.size()
                      << " config_keys=" << config.keys().size() << " substitutions=" << subs.substitutions.size() << "\n";
        } catch (const std::exception &e) {
            std::cout << "  [direct load_bbs_3mf] EXCEPTION: " << e.what() << "\n";
        }
        release_PlateData_list(plate_data);
        for (Preset *p : project_presets) delete p;
    }

    // (2) the GUI path: Model::read_from_archive(), exactly as Plater::priv::load_files() calls it
    PrusaFileParser prusa_parser;
    bool from_prusa = prusa_parser.check_3mf_from_prusa(path);
    std::cout << "  PrusaFileParser::check_3mf_from_prusa = " << from_prusa
              << (from_prusa ? "  -> read_from_archive uses the PrusaSlicer load_3mf() importer" : "  -> read_from_archive uses load_bbs_3mf()") << "\n";

    DynamicPrintConfig        config_loaded;
    ConfigSubstitutionContext config_substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs             plate_data;
    std::vector<Preset *>     project_presets;
    Semver                    file_version;
    En3mfType                 en_3mf_file_type = En3mfType::From_BBS;
    std::map<int, std::vector<std::string>> color_group_map;
    VolumeColorInfoMap        volume_color_data;
    Model model;
    try {
        model = Model::read_from_archive(path, &config_loaded, &config_substitutions, en_3mf_file_type, strategy, &plate_data, &project_presets,
                                         &file_version, nullptr, nullptr, &color_group_map, &volume_color_data);
    } catch (const std::exception &e) {
        std::cout << "  [read_from_archive] EXCEPTION: " << e.what() << "\n";
        std::cout << "  -> GUI would show an error dialog with this text and load nothing.\n";
        release_PlateData_list(plate_data);
        for (Preset *p : project_presets) delete p;
        return;
    }
    const char *ftype = en_3mf_file_type == En3mfType::From_BBS ? "From_BBS" : en_3mf_file_type == En3mfType::From_Prusa ? "From_Prusa" : "From_Other";
    std::cout << "  [read_from_archive] OK  en_3mf_file_type=" << ftype << " file_version=" << file_version.to_string()
              << " plate_data=" << plate_data.size() << " project_presets=" << project_presets.size()
              << " config substitutions=" << config_substitutions.substitutions.size()
              << " unrecognized keys=" << config_substitutions.unrecogized_keys.size() << "\n";
    for (const auto &s : config_substitutions.substitutions)
        std::cout << "    substitution: " << s.opt_def->opt_key << " '" << s.old_value << "' -> '" << s.new_value->serialize() << "'\n";
    print_config_summary(config_loaded);
    if (!color_group_map.empty() || !volume_color_data.empty())
        std::cout << "  color_group_map=" << color_group_map.size() << " volume_color_data=" << volume_color_data.size() << "\n";
    for (size_t i = 0; i < plate_data.size(); ++i) {
        std::cout << "  plate[" << i << "] index=" << plate_data[i]->plate_index << " objects_and_instances=" << plate_data[i]->objects_and_instances.size()
                  << " printer_model_id=" << plate_data[i]->printer_model_id << "\n";
    }

    std::cout << "  --- model as returned by read_from_archive ---\n";
    const std::string before_str = model_str(model);
    std::cout << before_str;

    // --- GUI post-processing (Plater.cpp load_files, 3MF branch) ---
    std::vector<std::string> notes;
    {
        // "1. add extruder for prusa model ... 2. add extruder for BBS or Other model if only import geometry"
        if (en_3mf_file_type == En3mfType::From_Prusa || (load_model && !load_config)) {
            std::set<int> extruderIds;
            for (ModelObject *o : model.objects) {
                if (o->config.option("extruder")) extruderIds.insert(o->config.extruder());
                for (auto volume : o->volumes) {
                    if (volume->config.option("extruder")) extruderIds.insert(volume->config.extruder());
                    for (int extruder : volume->get_extruders()) extruderIds.insert(extruder);
                }
            }
            int size = extruderIds.empty() ? 0 : *extruderIds.rbegin();
            notes.push_back("GUI raises project filament count to at least " + std::to_string(size) + " (max extruder id referenced)");
        }
        bool load_old_project = false;
        // Plater.cpp: if (!is_bbl_vendor_config(config_loaded, preset_bundle) && !check_project_config(config_loaded)) load_config = false;
        // is_bbl_vendor_config() needs the PresetBundle (printer_model must be a BBL vendor model); check_project_config() is pure:
        bool cfg_ok = true;
        if (!config_loaded.empty()) {
            auto *nd = config_loaded.option<ConfigOptionFloatsNullable>("nozzle_diameter");
            if (nd && nd->size() > 1) {
                const auto *opt = dynamic_cast<const ConfigOptionVectorBase *>(config_loaded.option("extruder_type"));
                cfg_ok = (opt ? opt->size() : 0) == nd->size();
            }
        }
        {
            auto *pm = config_loaded.option<ConfigOptionString>("printer_model");
            std::string pmv = pm ? pm->value : std::string();
            notes.push_back("check_project_config()=" + std::string(cfg_ok ? "true" : "false") + ", printer_model='" + pmv +
                            "' (is_bbl_vendor_config needs the preset bundle; true only if printer_model is a BBL vendor model)");
            if (!cfg_ok && pmv.empty()) {
                load_config = false;
                notes.push_back("-> load_config=false, notification 'The 3mf file has invalid config, load geometry data only'");
            }
        }
        Semver app_version = *(Semver::parse(SLIC3R_VERSION));
        if (en_3mf_file_type == En3mfType::From_Prusa) {
            load_config = false;
            notes.push_back("From_Prusa -> load_config=false, notification 'The 3mf is not from Bambu Lab, load geometry data only.'");
        } else if (load_config && file_version.maj() > app_version.maj()) {
            notes.push_back("file major version newer than app " + app_version.to_string() + " -> load_old_project=true (geometry only)");
            load_old_project = true;
        } else if (load_config && file_version > app_version) {
            notes.push_back("file version newer than app " + app_version.to_string() + " -> 'Newer3mfVersionDialog' (config still loaded)");
        } else if (load_config && config_loaded.empty()) {
            load_config = false;
            notes.push_back(std::string("config empty -> load_config=false, notification '") +
                            ((file_version.maj() == 0 && file_version.min() == 0 && file_version.patch() == 0)
                                 ? "The 3mf is not from Bambu Lab, load geometry data and color data only."
                                 : "The 3mf is generated by old Bambu Studio, load geometry data only.") + "' (object/volume configs are NOT reset)");
        } else if (!load_config) {
            notes.push_back("!load_config -> object/volume config reset, only 'extruder' kept");
            for (ModelObject *model_object : model.objects) {
                bool has_extruder = model_object->config.has("extruder");
                int  extruder_id  = -1;
                if (has_extruder) extruder_id = model_object->config.extruder();
                model_object->config.reset();
                if (has_extruder) model_object->config.set("extruder", extruder_id);
                for (ModelVolume *model_volume : model_object->volumes) {
                    has_extruder = model_volume->config.has("extruder");
                    if (has_extruder) extruder_id = model_volume->config.extruder();
                    model_volume->config.reset();
                    if (has_extruder) model_volume->config.set("extruder", extruder_id);
                }
            }
        }
        // is_project_file is set to true in the GUI only when the project config is actually applied
        bool is_project_file = load_config && !config_loaded.empty();
        notes.push_back(std::string("is_project_file=") + (is_project_file ? "true" : "false") +
                        (is_project_file ? " (assuming the project config passes is_bbl_vendor_config/check_project_config)" : ""));

        if (load_model) {
            if (!is_project_file && !load_old_project) {
                int deleted = model.removed_objects_with_zero_volume();
                if (deleted > 0) notes.push_back("removed " + std::to_string(deleted) + " zero-volume objects");
                if (model.looks_like_saved_in_meters()) { notes.push_back("looks_like_saved_in_meters -> convert_from_meters"); model.convert_from_meters(true); }
                else if (model.looks_like_imperial_units()) { notes.push_back("looks_like_imperial_units -> convert_from_imperial_units"); model.convert_from_imperial_units(true); }
            }
            if (!is_project_file && model.looks_like_multipart_object())
                notes.push_back("looks_like_multipart_object()=true -> GUI asks 'Multi-part object detected' (not answered here)");
        }
        for (ModelObject *mo : model.objects) mo->ensure_on_bed(is_project_file);
        if (!is_project_file && !load_old_project)
            model.center_instances_around_point(g_bed_center);
        Model scene;
        gui_load_model_objects(scene, model.objects, is_project_file);
        for (const std::string &n : notes) std::cout << "  GUI: " << n << "\n";
        std::cout << "  --- model as it would appear on the plate (after GUI post-processing + load_model_objects) ---\n";
        std::string after = model_str(scene);
        if (after == before_str)
            std::cout << "  (identical to the read_from_archive result above)\n";
        else
            std::cout << after;
    }
    release_PlateData_list(plate_data);
    for (Preset *p : project_presets) delete p;
}

// ----------------------------------------------------------------------------------------------
// STL
// ----------------------------------------------------------------------------------------------
// Per-file part of Plater::priv::load_files() for an STL with strategy LoadModel.
static Model gui_read_stl(const std::string &path, std::vector<std::string> &notes)
{
    PlateDataPtrs         plate_data;
    std::vector<Preset *> project_presets;
    bool                  is_xxx = false;
    Semver                file_version;
    Model model = Model::read_from_file(path, nullptr, nullptr, LoadStrategy::LoadModel, &plate_data, &project_presets, &is_xxx, &file_version,
                                        nullptr, nullptr, nullptr, 0, nullptr);
    for (auto obj : model.objects)
        if (obj->name.empty()) obj->name = fs::path(obj->input_file).filename().string();
    int deleted = model.removed_objects_with_zero_volume();
    if (deleted > 0) notes.push_back("removed " + std::to_string(deleted) + " zero-volume objects");
    if (model.looks_like_saved_in_meters()) { notes.push_back("looks_like_saved_in_meters -> convert_from_meters"); model.convert_from_meters(true); }
    else if (model.looks_like_imperial_units()) { notes.push_back("looks_like_imperial_units -> convert_from_imperial_units"); model.convert_from_imperial_units(true); }
    if (model.looks_like_multipart_object()) notes.push_back("looks_like_multipart_object()=true -> GUI would ask 'Multi-part object detected'");
    for (ModelObject *mo : model.objects) {
        mo->center_around_origin(false); // !type_3mf && !type_any_amf
        mo->ensure_on_bed(false);        // is_project_file == false
    }
    return model;
}

static void run_stl_single(const std::string &path)
{
    std::cout << "\n=== STL single file (Plater: SingleOther -> load_files({file}, LoadModel, ask_multi=false)) ===\n";
    std::cout << "  file: " << path << "\n";
    try {
        // Raw read_from_file result first
        Model raw = Model::read_from_file(path, nullptr, nullptr, LoadStrategy::LoadModel);
        std::cout << "  --- raw Model::read_from_file(path, nullptr, nullptr, LoadModel) ---\n";
        print_model(raw);
        std::vector<std::string> notes;
        Model model = gui_read_stl(path, notes);
        Model scene;
        gui_load_model_objects(scene, model.objects, false);
        for (const std::string &n : notes) std::cout << "  GUI: " << n << "\n";
        std::cout << "  --- on the plate after load_model_objects ---\n";
        print_model(scene);
    } catch (const std::exception &e) {
        std::cout << "  EXCEPTION: " << e.what() << "\n";
    }
}

static void run_stl_multi(const std::vector<std::string> &paths)
{
    std::cout << "\n=== STL multiple files (Plater: MultipleOther -> load_files(files, LoadModel, ask_multi=true)) ===\n";
    Model new_model;
    try {
        for (const std::string &p : paths) {
            std::vector<std::string> notes;
            Model model = gui_read_stl(p, notes);
            for (const std::string &n : notes) std::cout << "  GUI(" << fs::path(p).filename().string() << "): " << n << "\n";
            for (const ModelObject *mo : model.objects) new_model.add_object(*mo);
        }
    } catch (const std::exception &e) {
        std::cout << "  EXCEPTION: " << e.what() << "\n";
        return;
    }
    std::cout << "  merged new_model objects=" << new_model.objects.size() << "\n";
    if (new_model.objects.size() <= 1) {
        std::cout << "  (only one object -> no 'Load these files as a single object with multiple parts?' question)\n";
        return;
    }
    {
        std::cout << "\n  --- answer NO: separate objects (load_model_objects(new_model->objects)) ---\n";
        Model scene;
        gui_load_model_objects(scene, new_model.objects, false);
        print_model(scene, "    ");
    }
    {
        std::cout << "\n  --- answer YES: new_model->convert_multipart_object(" << g_filaments << ") then load_model_objects ---\n";
        Model merged = new_model;
        merged.convert_multipart_object(g_filaments);
        std::cout << "    after convert_multipart_object (before placement):\n";
        print_model(merged, "      ");
        Model scene;
        gui_load_model_objects(scene, merged.objects, false);
        std::cout << "    on the plate after load_model_objects:\n";
        print_model(scene, "      ");
        // relative positions check
        if (!scene.objects.empty() && scene.objects[0]->volumes.size() >= 2 && !scene.objects[0]->instances.empty()) {
            const ModelObject *o = scene.objects[0];
            std::cout << "    relative position check (world bbox min of each part minus part[0] min, vs. raw STL coords):\n";
            Transform3d inst = o->instances[0]->get_matrix();
            BoundingBoxf3 b0 = o->volumes[0]->mesh().transformed_bounding_box(inst * o->volumes[0]->get_matrix());
            for (size_t i = 0; i < o->volumes.size(); ++i) {
                BoundingBoxf3 bi = o->volumes[i]->mesh().transformed_bounding_box(inst * o->volumes[i]->get_matrix());
                std::cout << "      part[" << i << "] \"" << o->volumes[i]->name << "\" delta_min=" << v3(bi.min - b0.min) << "\n";
            }
            // raw coordinates from file
            std::vector<BoundingBoxf3> raw;
            for (const std::string &p : paths) {
                Model m = Model::read_from_file(p, nullptr, nullptr, LoadStrategy::LoadModel);
                raw.push_back(m.objects[0]->raw_mesh_bounding_box());
            }
            for (size_t i = 0; i < raw.size(); ++i)
                std::cout << "      raw file[" << i << "] " << fs::path(paths[i]).filename().string() << " bbox " << bbox_str(raw[i])
                          << " delta_min vs file[0]=" << v3(raw[i].min - raw[0].min) << "\n";
        }
    }
}

static void run_split(const std::string &path)
{
    std::cout << "\n=== Split to parts test on single STL: " << path << " ===\n";
    try {
        std::vector<std::string> notes;
        Model model = gui_read_stl(path, notes);
        Model scene;
        gui_load_model_objects(scene, model.objects, false);
        for (const std::string &n : notes) std::cout << "  GUI: " << n << "\n";
        std::cout << "  --- loaded (before split) ---\n";
        print_model(scene);
        if (scene.objects.empty()) return;
        ModelObject *mo = scene.objects[0];
        // ObjectList::split() (GUI_ObjectList.cpp ~3129): split volume 0 of the selected object
        ModelVolume *volume = mo->volumes[0];
        std::cout << "  ModelVolume::is_splittable() = " << volume->is_splittable() << "\n";
        Model scene_for_objects = scene; // copy for "split to objects"
        if (!volume->is_splittable()) {
            std::cout << "  GUI: 'The target object contains only one part and can not be split.'\n";
        } else {
            auto  world_tran = mo->instances[0]->get_transformation().get_matrix() * volume->get_matrix();
            float scale_det  = std::fabs(world_tran.matrix().block(0, 0, 3, 3).determinant());
            size_t n = volume->split(g_filaments, scale_det);
            std::cout << "  ModelVolume::split(" << g_filaments << ", " << scale_det << ") returned " << n << "; object now has "
                      << mo->volumes.size() << " volumes\n";
            std::cout << "  --- after Split to parts ---\n";
            print_model(scene);
        }
        // "Split to objects" (Plater::priv::split_object): split_volume first, then ModelObject::split
        {
            ModelObject *mo2 = scene_for_objects.objects[0];
            if (mo2->volumes.size() == 1 && mo2->volumes[0]->is_splittable()) {
                auto  world_tran = mo2->instances[0]->get_transformation().get_matrix() * mo2->volumes[0]->get_matrix();
                float scale_det  = std::fabs(world_tran.matrix().block(0, 0, 3, 3).determinant());
                mo2->volumes[0]->split(g_filaments, scale_det);
            }
            ModelObjectPtrs new_objects;
            mo2->split(&new_objects);
            std::cout << "  (for reference) Split to objects -> " << new_objects.size() << " objects\n";
        }
    } catch (const std::exception &e) {
        std::cout << "  EXCEPTION: " << e.what() << "\n";
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "usage: bbs_load file.3mf [...] | a.stl [b.stl ...] | --split one.stl\n";
        return 2;
    }
    const char *lvl = std::getenv("BBS_LOG");
    set_logging_level(lvl ? (unsigned)std::atoi(lvl) : 1);
    if (const char *x = std::getenv("BBS_BED_X")) g_bed_center.x() = std::atof(x);
    if (const char *y = std::getenv("BBS_BED_Y")) g_bed_center.y() = std::atof(y);
    if (const char *f = std::getenv("BBS_FILAMENTS")) g_filaments = (unsigned)std::atoi(f);

    // libslic3r needs a temp dir (3MF backup/aux extraction goes to <temp>/bamboo_model/...), a data dir and resources.
    fs::path exe_dir = fs::canonical(fs::path(argv[0])).parent_path();
    fs::path tmp     = exe_dir / "tmp";
    fs::create_directories(tmp);
    set_temporary_dir(tmp.string());
    set_data_dir((exe_dir / "datadir").string());
    fs::create_directories(exe_dir / "datadir");
#ifdef BBS_RESOURCES_DIR
    set_resources_dir(BBS_RESOURCES_DIR);
#endif

    std::vector<std::string> stls, tmfs;
    bool split_only = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--split") { split_only = true; continue; }
        if (boost::algorithm::iends_with(a, ".3mf")) tmfs.push_back(a);
        else stls.push_back(a);
    }
    std::cout << "bbs_load (libslic3r " << SLIC3R_VERSION << ")  bed_center=(" << g_bed_center.x() << "," << g_bed_center.y()
              << ") filaments=" << g_filaments << "\n";
    for (const std::string &p : tmfs) {
        std::time_t mt = fs::exists(p) ? fs::last_write_time(p) : 0;
        std::cout << "\n##### " << p << "  (" << (fs::exists(p) ? fs::file_size(p) : 0) << " bytes, mtime " << std::put_time(std::localtime(&mt), "%F %T") << ")\n";
        // Plater::load_project(): LoadModel|LoadConfig (Plater.cpp:20443); load_files adds LoadAuxiliary+CheckVersion
        run_3mf(p, "OPEN AS PROJECT (File>Open project / drop on empty plate)", LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
        // Plater::add_model() (Ctrl+I import, Plater.cpp:21001) and ProjectDropDialog 'import geometry' (Plater.cpp:22540): LoadModel
        run_3mf(p, "IMPORT GEOMETRY ONLY (File>Import / Ctrl+I, or drop + 'Import geometry only')", LoadStrategy::LoadModel);
    }
    if (split_only) {
        for (const std::string &p : stls) run_split(p);
    } else if (!stls.empty()) {
        for (const std::string &p : stls) {
            std::time_t mt = fs::exists(p) ? fs::last_write_time(p) : 0;
            std::cout << "\n##### " << p << "  (" << (fs::exists(p) ? fs::file_size(p) : 0) << " bytes, mtime " << std::put_time(std::localtime(&mt), "%F %T") << ")\n";
            run_stl_single(p);
        }
        if (stls.size() > 1) run_stl_multi(stls);
        if (stls.size() == 1) run_split(stls[0]);
    }
    std::cout << "\ndone.\n";
    return 0;
}
