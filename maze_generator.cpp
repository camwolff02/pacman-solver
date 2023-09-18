// Random Maze Generator for Pacman Maze Solver Visualizer by Ryan Nesius
// https://github.com/Decrypted-X/CS1D-Pacman
// code by Cameron Wolff

#include <bits/stdc++.h>

typedef std::vector<std::vector<int>> grid;

/****************************DEFINE_ALGORITHM_HERE*****************************/
enum State { open, wall, goal, visited};
const std::vector<int> dx {2, -2, 0, 0, 2, -2, 2, -2};
const std::vector<int> dy {0, 0, 2, -2, 2, -2, -2, 2};
std::vector<int> range {0, 1, 2, 3};

const bool in_bounds(int r, int c, int m, int n) {
    return r >= 0 and c >= 0 and r < m and c < n;
}

float dist(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

// DFS implementation of maze generation
std::tuple<grid, int, int, int, int> generate_maze(int rows, int cols) {
    std::random_device rd;  
    std::mt19937 rng ( rd() );  // doesn't work on windows for some reason
    //std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count()); 
    std::uniform_int_distribution<int> row_dist(0, rows-1);
    std::uniform_int_distribution<int> col_dist(0, cols-1);

    grid maze (rows, std::vector<int>(cols, wall));
    std::stack<std::pair<int, int>> frontier;
    int goal_r = row_dist(rng), goal_c = col_dist(rng);  // choose inital cell
    while (goal_r % 2 != 0) goal_r = row_dist(rng);  // make sure row is even
    while (goal_c % 2 != 0) goal_c = col_dist(rng);  // make sure col is even
    int row = goal_r, col = goal_c;  // seed end position

    maze[goal_r][goal_c] = goal;  // mark as visited 
    frontier.push({goal_r, goal_c});  // push to frontier

    while (not frontier.empty()) {  // while there are cells in the frontier
        auto [r, c] = frontier.top(); frontier.pop();  // pop cell and make it current cell

        std::shuffle(range.begin(), range.end(), rng);
        for (int num : range) {  // if current cell has any unvisited neighbors
            if (in_bounds(r+dx[num], c+dy[num], rows, cols)
            and maze[r+dx[num]][c+dy[num]] != open
            and maze[r+dx[num]][c+dy[num]] != goal) {
                frontier.push({r, c});  // push current cell 
                maze[r+dx[num]/2][c+dy[num]/2] = open;  // remove wall between current and new
                maze[r+dx[num]][c+dy[num]] = open;  // mark new as visited
                frontier.push({r+dx[num], c+dy[num]});  // push new 
                
                if (dist(goal_r, goal_c, r+dx[num], c+dy[num]) > dist(goal_r, goal_c, row, col))
                    row = r+dx[num], col = c+dy[num];
                break;  // stop searching for unvisited cell
            }
        }
    }

    std::cout << "start: " << row << ", " << col << std::endl;
    std::cout << "end:   " << goal_r << ", " << goal_c << std::endl;

    return {maze, row, col, goal_r, goal_c};
}
/******************************************************************************/
void print_maze(const grid& maze) {
    int m = maze.size(), n = maze[0].size();

    std::cout << "{";
    for (int i = 0; i < m; ++i) {
        std::cout << "{";
        for (int j = 0; j < n; ++j) 
            std::cout << maze[i][j] << (j < n-1 ? "," : "");
        std::cout << "}" << (i < n-1 ? ",\n" : "}\n");
    }

}

void write_maze_to_file(const grid& maze, int start_r, int start_c, int end_r, int end_c) {
    int m = maze.size(), n = maze[0].size();
    
    std::ofstream file;
    file.open("maze.csv");

    file << start_r << ',' << start_c << ',';
    file << end_r << ',' << end_c << ',';
    file << "n,";

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) 
           file << maze[i][j] << ',';
       file << "n,";
    }

    file.close();
}

std::tuple<grid, int, int, int, int> read_maze_from_file() {
    // open file and convert csv to parsable stringstream
    std::ifstream file ("maze.csv");
    std::string in;
    std::getline(file,in);
    std::stringstream in_stream(in);
    in.clear();

    // read in ints
    std::getline(in_stream, in, ',');
    int start_r = std::stoi(in);
    std::getline(in_stream, in, ',');
    int start_c = std::stoi(in);
    std::getline(in_stream, in, ',');
    int end_r = std::stoi(in);
    std::getline(in_stream, in, ',');
    int end_c = std::stoi(in);

    // read in grid
    grid maze;
    int row = -1;
    while (std::getline(in_stream, in, ',')) {
        if (in == "n") {
            ++row;
            maze.push_back(std::vector<int>());
        }
        else {
            maze[row].push_back(std::stoi(in));
        }
    }

    maze.pop_back();

    file.close();

    return {maze, start_r, start_c, end_r, end_c};
}

int main() {
    int rows, cols;

    do {
        std::cout << "enter rows and cols: ";
        std::cin >> rows;
        std::cin >> cols;
    } while (rows%2 == 0 and cols%2 == 0);

    auto [maze, start_r, start_c, end_r, end_c] = generate_maze(rows, cols);
    print_maze(maze);
    write_maze_to_file(maze, start_r, start_c, end_r, end_c);

    return 0;
}