#include <cstdint>
#include <iostream>

// A single cell in the board. Uses 8 bits. 5 are actually used.
// One bit for each wall direction, one bit for if a robot is present.
struct Cell {
    int8_t cell;

    bool has_robot() {
        return cell & 0b10000;
    }

    bool has_n_wall() {
        return cell & 0b1000;
    }

    bool has_w_wall() {
        return cell & 0b100;
    }

    bool has_e_wall() {
        return cell & 0b10;
    }

    bool has_s_wall() {
        return cell & 0b1;
    }
};

// The entire board as a 1D array of cells.
struct Board {
    Cell board[256];
};

// Position of all 4 robots.
struct Position {
    int32_t position;
    int8_t active_robot_position() {
        return position & 0xf;
    }
};

// Iteratively deepened DFS. Returns the number of steps and the optimal path.
void search() {
    // TODO: figure out signature
    // TODO: implement
}

void precompute(const Board& board, int8_t precomputed_map[256]) {
    // TODO: remove this dummy implementation, and then implement
    for (int i = 0; i < 256; i++) {
        precomputed_map[i] = i;
    }
}

int main() {
    std::cout << "Ricochet Robot solver" << std::endl;
    Board board;
    int8_t precomputed_map[256];
    precompute(board, precomputed_map);
    // std::cout << static_cast<int16_t>(precomputed_map[8]) << std::endl;
    return 0;
}