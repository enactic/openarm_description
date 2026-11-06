# Finger mesh assets

Coordinates are expressed in metres in the existing finger frames. Visual DAE files retain the contact pads, plastic body and metal mount as separate geometry groups.

| Representation | Inner finger triangles | Outer finger triangles | Location |
| --- | ---: | ---: | --- |
| Visual | 2486 | 2486 | `visual/finger_{inner,outer}.dae` |
| Whole collision | 2388 | 2374 | `collision/finger_{inner,outer}.stl` |
| Four-piece collision | 2580 | 2580 | `collision/splited/finger_{inner,outer}/` |
| Five-piece collision | 2590 | 2580 | `collision/splited/five_parts/finger_{inner,outer}/` |

The default `config/link/collisions.yaml` references the whole-finger STL files. Each split set is complete and uses the same frame as the whole mesh.

| Part | Four-piece set | Five-piece set |
| --- | --- | --- |
| `00` | Tip section | Tip section |
| `01` | Front section | Front section |
| `02` | Middle section | Middle section |
| `03` | Rear plastic and metal mount | Rear plastic |
| `04` | Included in `03` | Metal mount |

Collision STLs retain concave geometry. Each file is a closed, consistently wound single shell. The whole collision mesh is the union of the four-piece set, with internal cut faces removed.
