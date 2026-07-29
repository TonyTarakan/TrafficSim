#pragma once

namespace ts {

// TODO: graph optimization, refactor structs for SoA/DOD-ready
//
// PROPOSAL:
//
// // Узел — координаты + высота (для эстакад)
// struct Node {
//     double x, y, z;
// };
//
// // Ребро — направленное (от from к to), хранит геометрию и параметры дороги
// struct Edge {
//     NodeId from;
//     NodeId to;
//     float length;           // метры
//     float max_speed;        // м/с
//     uint8_t lane_count;     // количество полос
//     // Для кривых можно хранить контрольные точки, но пока оставим прямые
// };
//
// // Полоса — ссылается на ребро, имеет индекс внутри ребра (0..lane_count-1)
// struct Lane {
//     EdgeId edge_id;
//     uint8_t index;          // 0 = крайняя правая
// };

class ImmutableRoadGraph {
    // TODO
};

}  // namespace ts