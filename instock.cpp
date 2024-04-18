#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <fstream>
#include <sstream>
#include <tuple>

using namespace std;

struct Position {
    int x, y;
    bool operator==(const Position& o) const {
        return x == o.x && y == o.y;
    }
    bool operator<(const Position& o) const {
        return tie(x, y) < tie(o.x, o.y);
    }
};

struct King {
    Position current;
    Position target;
};

struct Chessboard {
    int size;
    vector<vector<bool>> restricted;
    vector<King> kings;

    Chessboard(int n) : size(n), restricted(n, vector<bool>(n, false)) {}
    
    void setRestricted(int x, int y) {
        restricted[x][y] = true;
    }

    bool isRestricted(int x, int y) {
        return restricted[x][y];
    }

    bool isValidPosition(int x, int y) {
        return x >= 0 && x < size && y >= 0 && y < size && !isRestricted(x, y);
    }

    bool isAdjacent(const Position& p1, const Position& p2) {
        return abs(p1.x - p2.x) <= 1 && abs(p1.y - p2.y) <= 1;
    }

    void addKing(int x_from, int y_from, int x_to, int y_to) {
        kings.push_back({{x_from, y_from}, {x_to, y_to}});
    }

    void bfsPathFinding() {
        // This function should implement the BFS and handle kings moving sequentially
    }

    void outputPlan() {
        // Output the movement plan
    }
};

int main() {
    ifstream file1("kings_positions.txt"), file2("chessboard_config.txt");
    int m, x, y;

    file2 >> m;
    Chessboard board(m);

    while (file2 >> x >> y) {
        board.setRestricted(x, y);
    }

    while (file1 >> x >> y >> m >> m) {
        board.addKing(x, y, m, m);
    }

    board.bfsPathFinding();
    board.outputPlan();

    return 0;
}
