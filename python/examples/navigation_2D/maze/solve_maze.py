from .df_maze import Maze
from .make_df_make import genMaze

from colorama import Fore

def escape(maze, rat_path, start, finish):
    current_cell = rat_path[len(rat_path) - 1]
    if current_cell == finish:
        return maze, rat_path

    if maze[current_cell[0] + 1][current_cell[1]] == 'c':
        maze[current_cell[0] + 1][current_cell[1]] = 'p'
        rat_path.append([current_cell[0] + 1, current_cell[1]])
        maze, rat_path = escape(maze, rat_path, start, finish)

    if maze[current_cell[0]][current_cell[1] + 1] == 'c':
        maze[current_cell[0]][current_cell[1] + 1] = 'p'
        rat_path.append([current_cell[0], current_cell[1] + 1])
        maze, rat_path = escape(maze, rat_path, start, finish)

    if maze[current_cell[0] - 1][current_cell[1]] == 'c':
        maze[current_cell[0] - 1][current_cell[1]] = 'p'
        rat_path.append([current_cell[0] - 1, current_cell[1]])
        maze, rat_path = escape(maze, rat_path, start, finish)

    if maze[current_cell[0]][current_cell[1] - 1] == 'c':
        
        maze[current_cell[0]][current_cell[1] - 1] = 'p'
        rat_path.append([current_cell[0], current_cell[1] - 1])
        maze, rat_path = escape(maze, rat_path, start, finish)

    # If we get here, this means that we made a wrong decision, so we need to
    # backtrack
    current_cell = rat_path[len(rat_path) - 1]
    if current_cell != finish:
        cell_to_remove = rat_path[len(rat_path) - 1]
        rat_path.remove(cell_to_remove)
        maze[cell_to_remove[0]][cell_to_remove[1]] = 'c'
    return maze, rat_path

def maze_solver(maze, start, finish):

    for i in range(0, len(maze)):
        for j in range(0, len(maze[0])):
            if i == start[0] and j == start[1] or i == finish[0] and j == finish[1]:
                print(Fore.MAGENTA, f'{maze[i][j]}', end=" ")
            elif maze[i][j] == 'u':
                print(Fore.WHITE, f'{maze[i][j]}', end=" ")
            elif maze[i][j] == 'c':
                print(Fore.GREEN, f'{maze[i][j]}', end=" ")
            elif maze[i][j] == 'p':
                print(Fore.BLUE, f'{maze[i][j]}', end=" ")
            else:
                print(Fore.RED, f'{maze[i][j]}', end=" ")
        print('\n')
    
def changeMazeRep(maze):
    #print(maze)
    s = maze
    s = s.replace("-", "w")
    s = s.replace("|", "w")
    s = s.replace("+", "w")
    s = s.replace(" ", "c")
    #print(s)
    s = s.split("\n")
    r = [list(x) for x in s] 
    return r

def solveMaze(nx = 15, ny = 15, ix = 0, iy=0):
    maze = genMaze(nx = nx, ny = ny, ix = ix, iy=iy)
    res = maze.__str__()
    mazeR = changeMazeRep(res)
    start = [1 , 1]
    finish = [len(mazeR) -2, len(mazeR[0]) -2]
    rat_path = [start]
    escape(mazeR, rat_path, start, finish)
    #print(rat_path)
    #maze_solver(mazeR, start, finish)
    return maze, rat_path

if __name__ == '__main__':
    solveMaze()

