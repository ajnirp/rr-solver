#include <cstdint>
#include <iostream>
#include <queue>
#include <utility>

template<typename T>
void print_array(T array[256]) {
    for (int i = 0; i < 256; i++) {
        // Newline in case we're on a new row.
        if (i > 0 and i % 16 == 0) {
            std::cout << std::endl;
        }
        std::cout << array[i] << "\t";
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

// Position of all 4 robots.
struct Position {
    int32_t position;

    // The active robot is the rightmost 8 bits of the position.
    int8_t active_robot_position() {
        return position & 0xff;
    }
};

// Iteratively deepened DFS. Returns the number of steps and the optimal path.
void search() {
    // TODO: figure out signature
    // TODO: implement
}

bool same_row(int cell1, int cell2) {
    return cell1/16 == cell2/16;
}

// Precompute the minimum number of moves to reach the target square from every
// other square, assuming the robot can stop moving at any time.
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

        // std::cout << "Seen so far:" << std::endl;
        // print_array(seen);
        // std::cout << std::endl;
    }
}

void setup_board(Cell board[256]) {
    add_central_walls(board);
}

void add_central_walls(Cell board[256]) {
    // N, W
    board[119].cell |= 0b1000;
    board[119].cell |= 0b0010;

    // N, E
    board[120].cell |= 0b1000;
    board[120].cell |= 0b0100;

    // S, W
    board[135].cell |= 0b0001;
    board[135].cell |= 0b0010;

    // S, E
    board[136].cell |= 0b0001;
    board[136].cell |= 0b0100;
}

int main() {
    std::cout << "Ricochet Robot solver" << std::endl;

    Cell board[256];
    setup_board(board);

    int precomputed_map[256] = {0};

    std::cout << "Precomputed map initialized to:" << std::endl;
    print_array(precomputed_map);
    std::cout << std::endl;

    precompute(board, 0, precomputed_map);
    std::cout << "Precomputed map is now:" << std::endl;
    print_array(precomputed_map);

    return 0;
}