# V2 body collision meshes

The body uses two native boxes and four convex mesh collisions in the body_link0 frame. Each STL uses metre coordinates in that frame. Native box origins are their center positions in `config/link/collisions.yaml`.

| Component | Collision shape | STL | Triangles |
| --- | --- | --- | ---: |
| Column | box | `collision/column.stl` | 12 |
| Diagonal brace | tapered_prism | `collision/diagonal_brace.stl` | 12 |
| Complete chest cover | convex_hull | `collision/chest_cover.stl` | 700 |
| Base plate | box | `collision/base_plate.stl` | 12 |
| Left mounting bracket | convex_hull | `collision/base_bracket_left.stl` | 16 |
| Right mounting bracket | convex_hull | `collision/base_bracket_right.stl` | 16 |

The column box measures 60 × 60 × 750 mm. The base plate box measures 250 × 190 × 8 mm. The mounting bracket hulls retain their clipped ends. The combined chest hull encloses both cover halves with a maximum sampled expansion of 0.570 mm relative to the original convex hull.
