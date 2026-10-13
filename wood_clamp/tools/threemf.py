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
    for name, mesh, extruder in parts:
        for x, y, z in mesh.vertices:
            vert_lines.append(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>')
        for a, b, c in mesh.faces:
            tri_lines.append(f'<triangle v1="{a + v_off}" v2="{b + v_off}" v3="{c + v_off}"/>')
        volumes.append((name, t_off, t_off + len(mesh.faces) - 1, extruder))
        v_off += len(mesh.vertices)
        t_off += len(mesh.faces)

    ox, oy, oz = offset
    model = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
        'xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06">\n'
        '<metadata name="slic3rpe:Version3mf">1</metadata>\n'
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
    for name, first, last, extruder in volumes:
        cfg += [f'<volume firstid="{first}" lastid="{last}">',
                f'<metadata type="volume" key="name" value={quoteattr(name)}/>',
                '<metadata type="volume" key="volume_type" value="ModelPart"/>',
                f'<metadata type="volume" key="extruder" value="{extruder}"/>',
                '</volume>']
    cfg += ['</object>', '</config>', '']

    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", model)
        z.writestr("Metadata/Slic3r_PE_model.config", "\n".join(cfg))
