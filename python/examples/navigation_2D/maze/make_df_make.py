from .df_maze import Maze


# Maze dimensions (ncols, nrows)
# Maze entry position

def genMaze(nx = 15, ny = 15, ix = 0, iy=0):

    maze = Maze(nx, ny, ix, iy)
    maze.add_begin_end = True
    maze.add_treasure = False
    maze.make_maze()
    return maze 


if __name__ == '__main__':
    maze = genMaze(nx = 15, ny = 15)
    maze.write_svg('maze.svg')