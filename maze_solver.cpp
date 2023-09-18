// Random Maze Generator for Pacman Maze Solver Visualizer by Ryan Nesius
// https://github.com/Decrypted-X/CS1D-Pacman
// code by Cameron Wolff

#include <bits/stdc++.h>

typedef std::vector<std::vector<int>> grid;
typedef std::vector<std::pair<int, int>> path;

/****************************DEFINE_FUNCTIONS_HERE*****************************/
//----------------------------------------------------------------------------//
// helper functions and data                                                  //
//----------------------------------------------------------------------------//
// header: path algorithm(const grid&, int, int)
enum State { OPEN, WALL, GOAL, VISITED};
int dt[] = {0, 1, 0, -1, 0};

bool in_bounds(int r, int c, int m, int n) {
    return r >= 0 and c >= 0 and r < m and c < n;
}

// use when we can move in 4 directions
float manhattan_dist(int x1, int y1, int x2, int y2) {
     return abs(x2 - x1) + abs(y2 - y1);
}

// use when we can move in 8 directions
float diagonal_dist(int x1, int y1, int x2, int y2, int d=1, int d2=sqrt(2)) {
    auto dx = abs(x2 - x1);
    auto dy = abs(y2 - y1);
 
    return d * (dx + dy) + (d2 - 2 * d) * std::min(dx, dy);
}

// use when we can move in any direction
float euclidean_dist(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

// for using hashed containers with std::pair<>
struct hash_pair {
  template<class T1, class T2>
  size_t operator()(const std::pair<T1, T2>& p) const {
    auto hash1 = std::hash<T1>{}(p.first);
    auto hash2 = std::hash<T2>{}(p.second);
  
    if(hash1 != hash2) {
      return hash1 ^ hash2;
    }
    return hash1;
  }
};

struct a_star_data {
    int r;
    int c;
    float g;
    float f;
    std::pair<int,int> from;

    // constructor
    a_star_data(int r, int c, float g, float f, std::pair<int,int> from) 
        : r{r}, c{c}, g{g}, f{f}, from{from} {}

    bool operator<(const a_star_data& rhs) const {
        return this->f < rhs.f;
    }

    bool operator>(const a_star_data& rhs) const {
        return this->f > rhs.f;
    }
} ;
//----------------------------------------------------------------------------//
// fast solution                                                              //
//----------------------------------------------------------------------------//
bool dfs(grid& maze, std::vector<std::pair<int, int>>& path, int r, int c, int m, int n) {
    path.push_back({r, c});
    if (maze[r][c] == GOAL) return true;
    maze[r][c] = VISITED;

    for (int i = 0; i < 4; ++i) 
        if (in_bounds(r+dt[i], c+dt[i+1], m, n) 
        and maze[r+dt[i]][c+dt[i+1]] != WALL
        and maze[r+dt[i]][c+dt[i+1]] != VISITED
        and dfs(maze, path, r+dt[i], c+dt[i+1], m, n))
            return true;
        
    path.erase(path.end());
    return false;
}

path dfs(const grid& maze, int r, int c, int end_r=-1, int end_c=-1) {
    path solution;
    grid copy = maze;
    if (dfs(copy, solution, r, c, maze.size(), maze[0].size())) 
        return solution;
    return {};
}
//----------------------------------------------------------------------------//
// shortest solution                                                          //
//----------------------------------------------------------------------------//
path a_star(const grid& init_maze, int start_r, int start_c, int goal_r, int goal_c) {
    grid maze = init_maze;
    int m = maze.size(), n = maze[0].size();

    // define heuristic for algorithm
    auto h = [=](int r, int c) {
        return manhattan_dist(goal_r, goal_c, r, c);
    };

    std::list<std::pair<int, int>> search_path;  // init "closed" list
    std::priority_queue<std::shared_ptr<a_star_data>> frontier;  // init "open" list
    // track data
    std::unordered_map<std::pair<int, int>, std::shared_ptr<a_star_data>, hash_pair> path_data;
    std::unordered_map<std::pair<int, int>, std::shared_ptr<a_star_data>, hash_pair> front_data;
    
    maze[start_r][start_c] = VISITED;  // mark starting position as visited
    frontier.push(std::make_shared<a_star_data>(start_r, start_c, 0, 0, std::make_pair(-1, -1)));  // put the starting node on the open list
    front_data[{start_r, start_c}] = frontier.top();

    while (not frontier.empty()) {  // while the open list is not empty
        // find and pop the node (q) with the least f (g + h(start_r, start_c))
        std::shared_ptr<a_star_data> q = frontier.top(); frontier.pop();
        auto [r, c, g, f, from] = *q;  

        if (h(r, c) != 0) {  // if this is not the solution
            // generate q' neighbors 
            for (int i = 0; i < 4; ++i) {  // for each successor
                int new_r = r+dt[i], new_c = c+dt[i+1];

                // if successor is not valid
                if (not in_bounds(new_r, new_c, m, n) or maze[new_r][new_c] == WALL or maze[new_r][new_c] == VISITED) 
                    continue;  // skip successor

                // else, computer g, h, and f for successor
                auto successor ( std::make_shared<a_star_data>(new_r, new_c, g+1, g+h(new_r, new_c), std::make_pair(r, c)));

                if (maze[new_r][new_c] == GOAL) {  // if successor is the goal, 
                    while (not frontier.empty()) frontier.pop();  // dump frontier
                    frontier.push(successor);  // add to frontier
                    break;  // stop search
                }

                // if a node with the same position as 
                // successor is in the OPEN list which has a 
                // lower g than successor, skip this successor
                if (front_data.contains({new_r, new_c}) and front_data[{new_r, new_c}]->g < successor->g)
                    continue;
                else front_data[{new_r, new_c}] = successor;

                // if a node with the same position as 
                // successor is in the CLOSED list which has a 
                // lower f than successor, skip this successor
                if (path_data.contains({new_r, new_c}) and path_data[{new_r, new_c}]->f < successor->f)
                    continue;
                else path_data[{new_r, new_c}] = successor;

                // else, add the node to the open list
                frontier.push(successor);
                front_data[{new_r, new_c}] = successor;
                maze[new_r][new_c] = VISITED;
            }
        }

        // add q to closed list
        search_path.push_back({r, c});
        path_data[{r, c}] = q;
    }

    // if the final position added is not the goal
    if (search_path.back().first != goal_r or search_path.back().second != goal_c)
        return {};  // we did not find a solution

    // now that we found goal, backtrace through path to find solution
    path solution;
    solution.push_back(search_path.back());

    while (solution.back() != std::make_pair(-1, -1)) 
        solution.push_back(path_data[solution.back()]->from);

    solution.pop_back();  // remove beginning marker

    return solution;
}

/******************************************************************************/

void print_solution(const grid& maze, int start_y, int start_x, int end_y, int end_x, path (*f)(const grid&, int, int, int, int)) {
    path path = (*f)(maze, start_y, start_x, end_y, end_x);
    
    if (path.empty()) {
        std::cout << "no path found\n";
    }
    else {
        std::cout << "{";
        for (int i = 0; i < path.size(); ++i) {
            auto [y, x] = path[i];
            std::cout << "{" << y << "," << x << "}" << (i == path.size() - 1 ? "" : ",");
        }
        std::cout << "}\n";
    }
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

    // perform cleanup
    maze.pop_back();
    file.close();

    return {maze, start_r, start_c, end_r, end_c};
}

/****************************DEFINE_PARAMETERS_HERE****************************/
int main() {
    std::unordered_map<std::string, path (*)(const grid&, int, int, int, int)> algos;  // search algos

/******************************ADD_ALGOS_HERE*******************************/
    algos["dfs"] = dfs;
    algos["a*"] = a_star;
/******************************************************************************/

    // Define search parameters
    auto [maze, start_y, start_x, goal_y, goal_x] = read_maze_from_file();
    std::string algo_name;

    std::cout << "Select algorithm:\n";
    for (const auto& [name, ptr] : algos) 
        std::cout << name << "\n";

    do {
        std::cout << ">>> ";
        std::getline(std::cin, algo_name);
    } while (not algos.contains(algo_name));

    // find and print solution
    print_solution(maze, start_y, start_x, goal_y, goal_x, algos[algo_name]);

    return 0;
}