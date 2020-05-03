#include <algorithm>
#include <cstdint>
#include <iostream>
#include <queue>
#include <stack>
#include <unordered_map>
#include <utility>
#include <vector>

typedef int32_t Position;

// Print a 16x16 grid, passed in as a 1D array with 256 elements.
template<typename T>
void print_grid(T array[256]) {
    for (int i = 0; i < 256; i++) {
        // Newline in case we're on a new row.
        if (i > 0 and i % 16 == 0) {
            std::cout << std::endl;
        }
        std::cout << array[i] << " ";
    }
    std::cout << std::endl;
}

void print_position(const Position position) {
    int a = (position & 0xff000000) >> 24;
    int b = (position & 0x00ff0000) >> 16;
    int c = (position & 0x0000ff00) >> 8;
    int d = (position & 0x000000ff);
    std::cout << "(" << a << ", " << b << ", " << c << ", " << d << ")";
}

void print_path(const std::vector<Position>& path) {
    for (auto position : path) {
        print_position(position);
        std::cout << " ";
    }
    std::cout << std::endl;
}

// A single cell in the board. Uses 8 bits. 5 are actually used.
// One bit for each wall direction, one bit for if a robot is present.
// Walls are stored twice, since a wall touches two cells.
struct Cell {
    int8_t cell;

    Cell() : cell(0) {}

    bool has_robot() const {
        return cell & 0b10000;
    }

    bool has_n_wall() const {
        return cell & 0b1000;
    }

    bool has_e_wall() const {
        return cell & 0b100;
    }

    bool has_w_wall() const {
        return cell & 0b10;
    }

    bool has_s_wall() const {
        return cell & 0b1;
    }
};

// The board is represented as a 1D array of cells.
// 0 is the top-leftmost cell.
// 255 is the bottom-rightmost cell.

int active_robot_cell(Position position) {
    return position & 0xff;
}

// Destructively copy a stack into a vector. Mutates both.
template<typename T, typename U>
void move_stack_to_vector(std::stack<std::pair<T,U>>* stack, std::vector<T>* vector) {
    while (!stack->empty()) {
        T top_elem = std::get<0>(stack->top());
        vector->push_back(top_elem);
        stack->pop();
    }
    std::reverse(vector->begin(), vector->end());
}

// Ascending sort.
void sort(int* a, int* b, int* c) {
    if (*a > *b) {
        std::swap(*a, *b);
    }
    if (*b > *c) {
        std::swap(*b, *c);
    }
    if (*a > *c) {
        std::swap(*a, *c);
    }
    if (*a > *b) {
        std::swap(*a, *b);
    }
}

Position create_position(int a, int b, int c, int d) {
    a = (a & 0xff) << 24;
    b = (b & 0xff) << 16;
    c = (c & 0xff) << 8;
    d = d & 0xff;
    return a | b | c | d;
}

// Non-destructively sort a Position and return a new one.
// Sorting a Position means sorting the first 3 (from the left) octets.
// These correspond to the non-active robots.
Position sorted_position(const Position position) {
    int a = (position &0xff000000) >> 24;
    int b = (position &0x00ff0000) >> 16;
    int c = (position &0x0000ff00) >> 8;
    sort(&a, &b, &c);
    return (a << 24) | (b << 16) | (c << 8) | (position & 0xff);
}

// Robot number `robot` is moved to `new_cell`. Compute and return the new position.
Position new_position(const Position curr_pos, const int robot, const int new_cell) {
    int shift = 8 * (3 - robot);
    int32_t mask = ~(0xff << shift);
    // Note: we grab the rightmost 8 bits of the new_cell just to be safe.
    return (curr_pos & mask) | ((int32_t(new_cell) & 0xff) << shift);
}

// In the robot_to_move parameter, 3 is the active robot (rightmost). 0, 1 and 2 (leftmost) are inactive robots.
Position make_move_north(Cell board[256], const Position curr_pos, const int robot_to_move) {
    const int shift = 8 * (3 - robot_to_move);
    const int robot_cell = (curr_pos & (0xff << shift)) >> shift;

    // Go as far as we can starting from the current cell.
    int neighbour = robot_cell;
    while (true) {
        if (board[neighbour].has_n_wall() or board[neighbour - 16].has_robot()) {
            break;
        }
        neighbour -= 16;
    }
    return new_position(curr_pos, robot_to_move, neighbour);
}

Position make_move_east(Cell board[256], const Position curr_pos, const int robot_to_move) {
    const int shift = 8 * (3 - robot_to_move);
    const int robot_cell = (curr_pos & (0xff << shift)) >> shift;

    // Go as far as we can starting from the current cell.
    int neighbour = robot_cell;
    while (true) {
        if (board[neighbour].has_e_wall() or board[neighbour + 1].has_robot()) {
            break;
        }
        neighbour++;
    }
    return new_position(curr_pos, robot_to_move, neighbour);
}

Position make_move_west(Cell board[256], const Position curr_pos, const int robot_to_move) {
    const int shift = 8 * (3 - robot_to_move);
    const int robot_cell = (curr_pos & (0xff << shift)) >> shift;

    // Go as far as we can starting from the current cell.
    int neighbour = robot_cell;
    while (true) {
        if (board[neighbour].has_w_wall() or board[neighbour - 1].has_robot()) {
            break;
        }
        neighbour--;
    }

    return new_position(curr_pos, robot_to_move, neighbour);
}

Position make_move_south(Cell board[256], const Position curr_pos, const int robot_to_move) {
    const int shift = 8 * (3 - robot_to_move);
    int robot_cell = (curr_pos & (0xff << shift)) >> shift;

    // Go as far as we can starting from the current cell.
    int neighbour = robot_cell;
    while (true) {
        if (board[neighbour].has_s_wall() or board[neighbour + 16].has_robot()) {
            break;
        }
        neighbour += 16;
    }
    return new_position(curr_pos, robot_to_move, neighbour);
}

Position make_move(Cell board[256], const Position curr_pos, const int robot_to_move, const int direction) {
    switch (direction) {
        case 0: return make_move_north(board, curr_pos, robot_to_move);
        case 1: return make_move_east(board, curr_pos, robot_to_move);
        case 2: return make_move_west(board, curr_pos, robot_to_move);
        case 3: return make_move_south(board, curr_pos, robot_to_move);
        default: return curr_pos;
    }
}

// Populate the robot-bit in the board variable using the current Position.
// Leftmost 8 bits of curr_pos = earliest non-active robot.
// Next 8 bits of curr_pos = next non-active robot.
// Next 8 bits of curr_pos = next non-active robot.
// Final 8 bits of curr_pos = active robot.
void populate_robot_bits(Cell board[256], const Position curr_pos) {
    const int robot_bit_mask = 0b10000;

    // First, zero out every robot bit in the board.
    for (int i = 0; i < 256; i++) {
        board[i].cell &= ~robot_bit_mask;
    }

    // Then, populate the robot bits for each robot-occupied cell.
    board[(curr_pos & 0xff000000) >> 24].cell |= robot_bit_mask;
    board[(curr_pos & 0x00ff0000) >> 16].cell |= robot_bit_mask;
    board[(curr_pos & 0x0000ff00) >> 8].cell |= robot_bit_mask;
    board[curr_pos & 0x000000ff].cell |= robot_bit_mask;
}

bool search_with_depth(Cell board[256], const Position start_pos, const int precomputed_map[256], const int goal, const int depth, std::unordered_map<Position, int>* moves_required, std::vector<Position>* result) {
    // Stack consists of (Position, moves_left)
    std::stack<std::pair<Position, int>> stack;
    stack.push(std::make_pair(start_pos, depth));

    while (!stack.empty()) {
        std::pair<Position, int> current = stack.top();
        stack.pop();

        const Position curr_pos = std::get<0>(current);
        const int moves_left = std::get<1>(current);

        // print_position(curr_pos);
        // std::cout << std::endl;

        if (moves_left < precomputed_map[active_robot_cell(curr_pos)]) {
            // We can't get to the goal fast enough. This Position is a dead end.
            // std::cout << "Ran out of moves" << std::endl;
            continue;
        }

        populate_robot_bits(board, curr_pos);

        // We didn't reach the goal, and we're out of moves.
        if (moves_left == 0) {
            std::cerr << "Error: Shouldn't be reached" << std::endl;
            break;
        }

        const int moves_used = depth - moves_left;

        for (int robot = 0; robot < 4; robot++) {
            for (int direction = 0; direction < 4; direction++) {
                // std::cout << "Making move for robot " << robot << " in direction " << direction << ". ";
                const Position new_pos = make_move(board, curr_pos, robot, direction);
                // std::cout << "New pos ";
                // print_position(new_pos);
                // std::cout << ". ";

                if (new_pos == curr_pos) {
                    // std::cout << "Same as old pos." << std::endl;
                    continue;
                }

                const Position sorted_pos = sorted_position(new_pos);
                // std::cout << "Sorted pos ";
                // print_position(sorted_pos);
                // std::cout << std::endl;

                auto already_found = moves_required->find(sorted_pos);
                if (already_found == moves_required->end() or moves_used + 1 < already_found->second) {
                    (*moves_required)[sorted_pos] = moves_used + 1;
                    stack.push(std::make_pair(new_pos, moves_left - 1));

                    // Victory check.
                    if (robot == 3 && active_robot_cell(new_pos) == goal) {
                        move_stack_to_vector(&stack, result);
                        return true;
                    }
                }
            }
        }
    }

    // Search failed, either because we ran out of moves or because there is no path to the goal.
    // TODO: is the second case even possible?
    return false;
}

// Iteratively deepened DFS.
bool search(Cell board[256], const Position start, const int precomputed_map[256], const int goal, std::vector<Position>* result) {
    // Map Positions to the lowest number of moves required to reach them from the start Position.
    std::unordered_map<Position, int> moves_required;

    int depth = 22;
    while (depth < 23) {
        std::cout << "Searching with depth = " << depth << std::endl;
        if (search_with_depth(board, start, precomputed_map, goal, depth, &moves_required, result)) {
            std::cout << "Found optimal path with depth = " << depth << std::endl;
            return true;
        }
        depth++;
    }

    return false;
}

bool same_row(int cell1, int cell2) {
    return cell1/16 == cell2/16;
}

// Precompute the minimum number of moves to reach the target square from every other square,
// assuming the robot can stop moving at any time, and assuming there are no other robots.
void precompute(const Cell board[256], int target_cell, int precomputed_map[256]) {
    // Precomputation is implemented as a BFS.
    bool seen[256] = {false};
    seen[target_cell] = true;

    // Cell index, number of moves
    std::queue<std::pair<int, int> > queue;
    queue.push(std::make_pair(target_cell, 0));

    while (!queue.empty()) {
        const std::pair<int, int> current = queue.front();
        queue.pop();
        const int curr_cell = std::get<0>(current);
        const int num_moves = std::get<1>(current);
        precomputed_map[curr_cell] = num_moves;

        // Push each neighbour into the queue, and then increment num_moves

        // N neighbours
        int neighbour_north = curr_cell - 16;
        while (neighbour_north >= 0 and !board[neighbour_north].has_s_wall()) {
            if (!seen[neighbour_north]) {
                queue.push(std::make_pair(neighbour_north, num_moves+1));
                seen[neighbour_north] = true;
            }
            neighbour_north -= 16;
        }

        // S neighbours
        int neighbour_south = curr_cell + 16;
        while (neighbour_south <= 255 and !board[neighbour_south].has_n_wall()) {
            if (!seen[neighbour_south]) {
                queue.push(std::make_pair(neighbour_south, num_moves+1));
                seen[neighbour_south] = true;
            }
            neighbour_south += 16;
        }

        // E neighbours
        int neighbour_east = curr_cell + 1;
        while (neighbour_east <= 255 and same_row(neighbour_east, curr_cell) and !board[neighbour_east].has_w_wall()) {
            if (!seen[neighbour_east]) {
                queue.push(std::make_pair(neighbour_east, num_moves+1));
                seen[neighbour_east] = true;
            }
            neighbour_east++;
        }

        // W neighbours
        int neighbour_west = curr_cell - 1;
        while (neighbour_west >= 0 and same_row(neighbour_west, curr_cell) and !board[neighbour_west].has_e_wall()) {
            if (!seen[neighbour_west]) {
                queue.push(std::make_pair(neighbour_west, num_moves+1));
                seen[neighbour_west] = true;
            }
            neighbour_west--;
        }
    }
}

void add_wall(Cell board[256], int cell1, int cell2) {
    if (cell1 == cell2) {
        std::cerr << "Invalid wall: cell1 = cell2 = " << cell1 << std::endl;
        return;
    }

    // Ensure cell1 < cell2
    if (cell1 > cell2) {
        std::swap(cell1, cell2);
    }

    if (same_row(cell1, cell2) and cell2 - cell1 == 1) {
        // Vertical wall case
        board[cell1].cell |= 0b0100;
        board[cell2].cell |= 0b0010;
    } else if (cell2 - cell1 == 16) {
        // Horizontal wall case
        board[cell1].cell |= 0b0001;
        board[cell2].cell |= 0b1000;
    } else {
        // No common edge
        std::cerr << "This cell pair does not share an edge: (cell1, cell2) == (" << cell1 << ", " << cell2 << ")";
        return;
    }
}

void add_central_walls(Cell board[256]) {
    // cell 119 has walls at N, W
    add_wall(board, 119, 118);
    add_wall(board, 119, 103);

    // cell 120 has walls at N, E
    add_wall(board, 120, 121);
    add_wall(board, 120, 104);

    // cell 135 has walls at S, W
    add_wall(board, 135, 134);
    add_wall(board, 135, 151);

    // cell 136 has walls at S, E
    add_wall(board, 136, 137);
    add_wall(board, 136, 152);
}

void add_wall_north(Cell board[256], int cell) {
    // Northern edge.
    if (cell < 16) {
        board[cell].cell |= 0b1000;
        return;
    }
    add_wall(board, cell, cell-16);
}

void add_wall_east(Cell board[256], int cell) {
    // Eastern edge.
    if (cell % 16 == 15) {
        board[cell].cell |= 0b0100;
        return;
    }
    add_wall(board, cell, cell+1);
}

void add_wall_west(Cell board[256], int cell) {
    // Western edge.
    if (cell % 16 == 0) {
        board[cell].cell |= 0b0010;
        return;
    }
    add_wall(board, cell, cell-1);
}

void add_wall_south(Cell board[256], int cell) {
    // Southern edge.
    if (cell > 239) {
        board[cell].cell |= 0b0001;
        return;
    }
    add_wall(board, cell, cell+16);
}

void add_northern_edge_walls(Cell board[256]) {
    for (int i = 0; i < 16; i++) {
        add_wall_north(board, i);
    }
}

void add_eastern_edge_walls(Cell board[256]) {
    for (int i = 0; i < 16; i++) {
        add_wall_east(board, i*16 + 15);
    }
}

void add_western_edge_walls(Cell board[256]) {
    for (int i = 0; i < 16; i++) {
        add_wall_west(board, i*16);
    }
}

void add_southern_edge_walls(Cell board[256]) {
    for (int i = 0; i < 16; i++) {
        add_wall_south(board, 15*16 + i);
    }
}

void add_border_walls(Cell board[256]) {
    add_northern_edge_walls(board);
    add_eastern_edge_walls(board);
    add_western_edge_walls(board);
    add_southern_edge_walls(board);
}

// // Wall configurations: NW, NE, SW, SE
// void add_nw_wall_pair(Cell board[256], int cell) {
//     add_wall_north(board, cell);
//     add_wall_west(board, cell);
// }
// void add_ne_wall_pair(Cell board[256], int cell) {
//     add_wall_north(board, cell);
//     add_wall_east(board, cell);
// }
// void add_sw_wall_pair(Cell board[256], int cell) {
//     add_wall_south(board, cell);
//     add_wall_west(board, cell);
// }
// void add_se_wall_pair(Cell board[256], int cell) {
//     add_wall_south(board, cell);
//     add_wall_east(board, cell);
// }

void setup_board(Cell board[256]) {
    add_border_walls(board);
    add_central_walls(board);

    // std::vector<int> nw = {4, 10, 30, 69, }

    int list1[20] = {3, 9, 21, 29, 42, 49, 68, 82, 109, 122, 147, 157, 170, 176, 198, 201, 225, 237, 245, 250};
    for (int i = 0; i < 20; i++) {
        add_wall_east(board, list1[i]);
    }

    int list2[20] = {14, 22, 33, 43, 53, 63, 82, 96, 106, 109, 131, 142, 171, 175, 177, 185, 198, 208, 210, 237};
    for (int i = 0; i < 20; i++) {
        add_wall_south(board, list2[i]);
    }
}

int main() {
    std::cout << "Ricochet Robot solver" << std::endl;

    Cell board[256];
    setup_board(board);

    int precomputed_map[256] = {0};
    const int goal = 201;

    precompute(board, goal, precomputed_map);
    std::cout << "Precomputed map:" << std::endl;
    print_grid(precomputed_map);

    Position start = create_position(12, 28, 241, 15);
    
    std::vector<Position> result;

    if (search(board, start, precomputed_map, goal, &result)) {
        std::cout << "Found a path." << std::endl;
        print_path(result);
        return 0;
    } else {
        std::cout << "Failed to find a path" << std::endl;
        return 1;
    }
}
