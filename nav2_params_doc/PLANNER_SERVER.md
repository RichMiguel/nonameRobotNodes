# Planner Server Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `planner_server -> ros__parameters`

Planner Server menghitung jalur global bebas tabrakan dari posisi robot saat ini ke titik tujuan akhir pada `global_costmap`.

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `expected_planner_frequency` | Float | `1.0` | `20.0` | Frekuensi pembaruan rute global (Hz). Disetel ke 1.0 Hz (1 detik sekali). Pada lingkungan terstruktur rumah sakit, rute global tidak perlu dihitung ulang terlalu cepat, sehingga menghemat daya CPU SBC secara drastis. |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu nyata hardware. |
| `planner_plugins` | Array of String | `["GridBased"]` | `["GridBased"]` | Plugin algoritma path planning yang didaftarkan. |
| `GridBased.plugin` | String | `"nav2_navfn_planner/NavfnPlanner"` | `"nav2_navfn_planner/NavfnPlanner"` | Algoritma path planner Navfn standar (berbasis ekspansi Dijkstra/Wavefront teruji). |
| `GridBased.tolerance` | Float | `0.5` | `0.5` | Toleransi jarak (meter, $50\text{ cm}$) jika titik target persis berada di dalam rintangan; planner akan mencari titik bebas terdekat dalam radius toleransi ini. |
| `GridBased.use_astar` | Boolean | `false` | `false` | Jika `false`, menggunakan algoritma Dijkstra. Dijkstra menjamin penemuan jalur global optimal terpendek pada grid statis. |
| `GridBased.allow_unknown` | Boolean | `true` | `true` | Mengizinkan perencana jalur melintasi area yang belum sepenuhnya dieksplorasi jika diperlukan. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Frekuensi 1.0 Hz**: Cukup untuk mendeteksi pembaruan rute jika ada lorong yang tertutup total, tanpa menyebabkan fluktuasi bentuk jalur global yang dapat membuat pergerakan robot tersentak-sentak.
2. **Dijkstra Reliability**: Dijkstra menjamin jalur terpendek tanpa risiko jebakan lokal heuristik pada denah lorong rumah sakit yang bercabang banyak.
