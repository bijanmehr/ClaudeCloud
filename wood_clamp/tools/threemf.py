"""Minimal multi-material 3MF writer.

Writes one printable object made of several parts ("volumes"), each with its
own extruder/filament slot, in the PrusaSlicer project dialect
(Metadata/Slic3r_PE_model.config). PrusaSlicer reads the per-part extruder
assignment directly; Bambu Studio / OrcaSlicer import the same file as one
object with separate parts, and you pick the filament per part in the
Objects list.
"""
import zipfile
from xml.sax.saxutils import quoteattr

_CONTENT_TYPES = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
    '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
    '</Types>\n'
)

_RELS = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
    '</Relationships>\n'
)


def write_multimaterial_3mf(path, object_name, parts, offset=(0.0, 0.0, 0.0)):
    """parts: list of (part_name, trimesh.Trimesh, extruder_number starting at 1).

    All parts keep their coordinates, so parts modelled in the same frame stay
    registered. `offset` is applied as the build-item transform (e.g. to move
    the object onto the middle of the bed).
    """
    vert_lines, tri_lines, volumes = [], [], []
    v_off = t_off = 0
    for part in parts:
        name, mesh, extruder = part[:3]
        for x, y, z in mesh.vertices:
            vert_lines.append(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>')
        for a, b, c in mesh.faces:
            tri_lines.append(f'<triangle v1="{a + v_off}" v2="{b + v_off}" v3="{c + v_off}"/>')
        vtype = {"support_blocker": "SupportBlocker"}.get(part[3] if len(part) > 3 else "", "ModelPart")
        volumes.append((name, t_off, t_off + len(mesh.faces) - 1, extruder, vtype))
        v_off += len(mesh.vertices)
        t_off += len(mesh.faces)

    ox, oy, oz = offset
    model = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
        'xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06">\n'
        '<metadata name="slic3rpe:Version3mf">1</metadata>\n'
        # Bambu Studio only reads Slic3r_PE_model.config (the per-part
        # filaments and the support blocker) when this names PrusaSlicer;
        # otherwise it loads all parts as one mesh on filament 1
        '<metadata name="Application">PrusaSlicer-2.7.2</metadata>\n'
        f'<metadata name="Title">{object_name}</metadata>\n'
        '<resources>\n<object id="1" type="model">\n<mesh>\n<vertices>\n'
        + "\n".join(vert_lines)
        + '\n</vertices>\n<triangles>\n'
        + "\n".join(tri_lines)
        + '\n</triangles>\n</mesh>\n</object>\n</resources>\n<build>\n'
        f'<item objectid="1" transform="1 0 0 0 1 0 0 0 1 {ox:.4f} {oy:.4f} {oz:.4f}" printable="1"/>\n'
        '</build>\n</model>\n'
    )

    cfg = ['<?xml version="1.0" encoding="UTF-8"?>', '<config>',
           '<object id="1" instances_count="1">',
           f'<metadata type="object" key="name" value={quoteattr(object_name)}/>']
    for name, first, last, extruder, vtype in volumes:
        cfg += [f'<volume firstid="{first}" lastid="{last}">',
                f'<metadata type="volume" key="name" value={quoteattr(name)}/>',
                f'<metadata type="volume" key="volume_type" value="{vtype}"/>',
                f'<metadata type="volume" key="extruder" value="{extruder}"/>',
                '</volume>']
    cfg += ['</object>', '</config>', '']

    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", model)
        z.writestr("Metadata/Slic3r_PE_model.config", "\n".join(cfg))


_BBS_NS = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
           'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
           'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06"')
_IDENT = "1 0 0 0 1 0 0 0 1 0 0 0"
_IDENT16 = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"


def _mesh_xml(mesh):
    v = "\n".join(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>' for x, y, z in mesh.vertices)
    t = "\n".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in mesh.faces)
    return f"<mesh>\n<vertices>\n{v}\n</vertices>\n<triangles>\n{t}\n</triangles>\n</mesh>"


def write_bambu_3mf(path, object_name, parts, offset=(0.0, 0.0, 0.0)):
    """One object made of several parts, laid out the way Bambu Studio /
    OrcaSlicer save their own projects: the parts live in
    3D/Objects/object_1.model, the build object assembles them as
    components, and Metadata/model_settings.config gives each part its
    filament number (extruder 1, 2, ... = filament slots in the project).

    parts: list of (part_name, trimesh.Trimesh, filament_number starting at 1
    [, subtype]) where subtype is "normal_part" (default) or "support_blocker".
    """
    sub = "/3D/Objects/object_1.model"
    objs = "\n".join(f'<object id="{i + 1}" type="model">\n{_mesh_xml(p[1])}\n</object>' for i, p in enumerate(parts))
    sub_model = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {_BBS_NS} '
                 f'requiredextensions="p">\n<resources>\n{objs}\n</resources>\n<build/>\n</model>\n')
    main_id = len(parts) + 1
    comps = "\n".join(f'<component p:path="{sub}" objectid="{i + 1}" transform="{_IDENT}"/>' for i in range(len(parts)))
    ox, oy, oz = offset
    main = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {_BBS_NS} '
            f'requiredextensions="p">\n<metadata name="Title">{object_name}</metadata>\n'
            f'<resources>\n<object id="{main_id}" type="model">\n<components>\n{comps}\n</components>\n</object>\n</resources>\n'
            f'<build>\n<item objectid="{main_id}" transform="1 0 0 0 1 0 0 0 1 {ox:.4f} {oy:.4f} {oz:.4f}" printable="1"/>\n'
            f'</build>\n</model>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            f'<Relationship Target="{sub}" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
            '</Relationships>\n')
    cfg = ['<?xml version="1.0" encoding="UTF-8"?>', "<config>", f'  <object id="{main_id}">',
           f'    <metadata key="name" value={quoteattr(object_name)}/>',
           f'    <metadata key="extruder" value="{parts[0][2]}"/>']
    for i, p in enumerate(parts):
        name, filament = p[0], p[2]
        subtype = p[3] if len(p) > 3 else "normal_part"
        cfg += [f'    <part id="{i + 1}" subtype="{subtype}">',
                f'      <metadata key="name" value={quoteattr(name)}/>',
                f'      <metadata key="matrix" value="{_IDENT16}"/>',
                f'      <metadata key="extruder" value="{filament}"/>',
                "    </part>"]
    cfg += ["  </object>", "</config>", ""]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", main)
        z.writestr("3D/_rels/3dmodel.model.rels", rels)
        z.writestr(sub.lstrip("/"), sub_model)
        z.writestr("Metadata/model_settings.config", "\n".join(cfg))


def write_bambu_3mf_multi(path, title, objects):
    """Several objects on one plate, each made of parts, Bambu Studio layout.

    objects: list of dicts with
      name     object name
      parts    list of (part_name, trimesh.Trimesh, filament [, subtype])
      offset   (x, y, z) build position
      settings optional {option_key: value} saved as per-object settings
               (Bambu Studio applies them when the file is opened as a project)
    Every object gets its own 3D/Objects/object_N.model; ids are unique
    across the whole file.
    """
    files, comps_xml, cfg = {}, [], ['<?xml version="1.0" encoding="UTF-8"?>', "<config>"]
    next_id = 1
    sub_ids = []
    for k, ob in enumerate(objects):
        sub = f"/3D/Objects/object_{k + 1}.model"
        ids = list(range(next_id, next_id + len(ob["parts"])))
        next_id += len(ob["parts"])
        sub_ids.append((sub, ids))
        objs = "\n".join(f'<object id="{i}" type="model">\n{_mesh_xml(p[1])}\n</object>' for i, p in zip(ids, ob["parts"]))
        files[sub] = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {_BBS_NS} '
                      f'requiredextensions="p">\n<resources>\n{objs}\n</resources>\n<build/>\n</model>\n')
    main_objs, items = [], []
    for k, ob in enumerate(objects):
        sub, ids = sub_ids[k]
        oid = next_id + k
        comps = "\n".join(f'<component p:path="{sub}" objectid="{i}" transform="{_IDENT}"/>' for i in ids)
        main_objs.append(f'<object id="{oid}" type="model">\n<components>\n{comps}\n</components>\n</object>')
        ox, oy, oz = ob["offset"]
        items.append(f'<item objectid="{oid}" transform="1 0 0 0 1 0 0 0 1 {ox:.4f} {oy:.4f} {oz:.4f}" printable="1"/>')
        cfg += [f'  <object id="{oid}">', f'    <metadata key="name" value={quoteattr(ob["name"])}/>',
                f'    <metadata key="extruder" value="{ob["parts"][0][2]}"/>']
        for key, val in ob.get("settings", {}).items():
            cfg.append(f'    <metadata key="{key}" value={quoteattr(str(val))}/>')
        for i, p in zip(ids, ob["parts"]):
            subtype = p[3] if len(p) > 3 else "normal_part"
            cfg += [f'    <part id="{i}" subtype="{subtype}">',
                    f'      <metadata key="name" value={quoteattr(p[0])}/>',
                    f'      <metadata key="matrix" value="{_IDENT16}"/>',
                    f'      <metadata key="extruder" value="{p[2]}"/>',
                    "    </part>"]
        cfg.append("  </object>")
    cfg += ["</config>", ""]
    main = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {_BBS_NS} '
            f'requiredextensions="p">\n<metadata name="Title">{title}</metadata>\n'
            f'<resources>\n' + "\n".join(main_objs) + '\n</resources>\n<build>\n' + "\n".join(items) +
            '\n</build>\n</model>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' +
            "".join(f'<Relationship Target="{sub}" Id="rel-{k + 1}" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
                    for k, (sub, _) in enumerate(sub_ids)) + '</Relationships>\n')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", main)
        z.writestr("3D/_rels/3dmodel.model.rels", rels)
        for sub, xml in files.items():
            z.writestr(sub.lstrip("/"), xml)
        z.writestr("Metadata/model_settings.config", "\n".join(cfg))


def check_bambu_3mf(path):
    """Re-read the file the way Bambu Studio's importer resolves it and
    return [(part name, filament, triangle count)] - a structural self-test."""
    import xml.etree.ElementTree as ET
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02",
          "p": "http://schemas.microsoft.com/3dmanufacturing/production/2015/06"}
    with zipfile.ZipFile(path) as z:
        main = ET.fromstring(z.read("3D/3dmodel.model"))
        rels = ET.fromstring(z.read("3D/_rels/3dmodel.model.rels"))
        targets = [r.get("Target") for r in rels]
        subs = {t: ET.fromstring(z.read(t.lstrip("/"))) for t in targets}
        cfg = ET.fromstring(z.read("Metadata/model_settings.config"))
    item = main.find("m:build/m:item", ns)
    obj = [o for o in main.find("m:resources", ns) if o.get("id") == item.get("objectid")][0]
    cfg_obj = [o for o in cfg if o.get("id") == obj.get("id")][0]
    out = []
    for comp in obj.find("m:components", ns):
        sub_path = comp.get("{%s}path" % ns["p"])
        sub_obj = [o for o in subs[sub_path].find("m:resources", ns) if o.get("id") == comp.get("objectid")][0]
        part = [p for p in cfg_obj.findall("part") if p.get("id") == comp.get("objectid")][0]
        meta = {m.get("key"): m.get("value") for m in part.findall("metadata")}
        ntri = len(sub_obj.find("m:mesh/m:triangles", ns))
        out.append((meta["name"], int(meta["extruder"]), ntri, part.get("subtype")))
    return out
