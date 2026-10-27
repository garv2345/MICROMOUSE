# maze.py

# Bitmasks for wall storage
# Each cell is an integer where the lower 4 bits represent walls.
# 1 means there IS a wall. 0 means NO wall.
NORTH = 0b0001
EAST  = 0b0010
SOUTH = 0b0100
WEST  = 0b1000

# To make coordinate changes easy based on heading
# 0: North, 1: East, 2: South, 3: West
class Heading:
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    # DX, DY vectors. Index matches the heading above.
    D_X = [0, 1, 0, -1]
    D_Y = [1, 0, -1, 0]

class Maze:
    def __init__(self, width=8, height=8):
        self.width = width
        self.height = height
        
        # Grid of walls. Start with 0 (no walls known)
        # But we will initialize the outer borders.
        self.walls = [[0 for _ in range(self.height)] for _ in range(self.width)]
        
        # Grid of distances for Flood Fill.
        # 255 represents infinity (unreachable)
        self.distances = [[255 for _ in range(self.height)] for _ in range(self.width)]
        
        self._init_perimeters()
        
    def _init_perimeters(self):
        """Add walls to the absolute outer borders of the 16x16 maze so flood fill doesn't leave the map."""
        for x in range(self.width):
            self.add_wall(x, 0, SOUTH)
            self.add_wall(x, self.height - 1, NORTH)
            
        for y in range(self.height):
            self.add_wall(0, y, WEST)
            self.add_wall(self.width - 1, y, EAST)

    def has_wall(self, x, y, direction):
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True # Everything outside is a wall
        return (self.walls[x][y] & direction) != 0

    def add_wall(self, x, y, direction):
        """Adds a wall at (x,y) in the given direction. Also adds the corresponding wall to the neighboring cell."""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
            
        self.walls[x][y] |= direction
        
        # Add the reciprocal wall to the neighbor
        if direction == NORTH and y + 1 < self.height:
            self.walls[x][y + 1] |= SOUTH
        elif direction == EAST and x + 1 < self.width:
            self.walls[x + 1][y] |= WEST
        elif direction == SOUTH and y - 1 >= 0:
            self.walls[x][y - 1] |= NORTH
        elif direction == WEST and x - 1 >= 0:
            self.walls[x - 1][y] |= EAST

    def update_cell_walls(self, x, y, current_heading, wall_left, wall_front, wall_right):
        """
        Takes sensor readings (True/False) and updates the maze structure based on current absolute heading.
        """
        # Figure out absolute directions based on current heading
        # If heading is NORTH (0): Front=NORTH, Right=EAST, Left=WEST
        # If heading is EAST (1): Front=EAST, Right=SOUTH, Left=NORTH
        
        dir_front = (1 << current_heading)
        dir_right = (1 << ((current_heading + 1) % 4))
        dir_left  = (1 << ((current_heading + 3) % 4))
        
        if wall_front:
            self.add_wall(x, y, dir_front)
        if wall_right:
            self.add_wall(x, y, dir_right)
        if wall_left:
            self.add_wall(x, y, dir_left)

    def flood_fill(self, target_x, target_y):
        """
        Recalculates the distances of all cells to the target(s).
        Uses a breadth-first search (BFS) approach.
        """
        # 1. Reset all distances to 255
        for x in range(self.width):
            for y in range(self.height):
                self.distances[x][y] = 255
                
        # 2. Queue for BFS. Starts with the target cell at distance 0
        queue = [(target_x, target_y)]
        head = 0
        self.distances[target_x][target_y] = 0
        
        # 3. Process the queue (index-based to avoid O(n) list.pop(0))
        while head < len(queue):
            current_x, current_y = queue[head]
            head += 1
            current_dist = self.distances[current_x][current_y]
            
            # Check all 4 neighbors
            for heading in [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]:
                # If there is NO wall blocking us in this direction
                if not self.has_wall(current_x, current_y, 1 << heading):
                    # Calculate neighbor coordinates
                    nx = current_x + Heading.D_X[heading]
                    ny = current_y + Heading.D_Y[heading]
                    
                    # Ensure neighbor is within bounds
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # If neighbor hasn't been visited (distance is 255)
                        if self.distances[nx][ny] == 255:
                            self.distances[nx][ny] = current_dist + 1
                            queue.append((nx, ny))

    def get_best_move(self, current_x, current_y, current_heading):
        """
        Looks at the current cell and returns the heading we should face to go to the 
        neighbor with the absolute lowest distance.
        """
        best_distance = 255
        best_heading = current_heading # Default to keep going same direction if stuck
        
        # Evaluate all open neighbors
        for heading in [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]:
            if not self.has_wall(current_x, current_y, 1 << heading):
                nx = current_x + Heading.D_X[heading]
                ny = current_y + Heading.D_Y[heading]
                
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    neighbor_dist = self.distances[nx][ny]
                    
                    # If this neighbor is closer to target than our best so far
                    if neighbor_dist < best_distance:
                        best_distance = neighbor_dist
                        best_heading = heading
                        
        return best_heading

    def save_to_file(self, filename="maze_map.txt"):
        """Save the maze wall data to a file on the Pico's flash."""
        with open(filename, "w") as f:
            f.write(f"{self.width},{self.height}\n")
            for x in range(self.width):
                row = ",".join(str(self.walls[x][y]) for y in range(self.height))
                f.write(row + "\n")
        print(f"Maze saved to {filename}")

    def load_from_file(self, filename="maze_map.txt"):
        """Load the maze wall data from a file on the Pico's flash."""
        try:
            with open(filename, "r") as f:
                header = f.readline().strip().split(",")
                w, h = int(header[0]), int(header[1])
                if w != self.width or h != self.height:
                    print("Maze size mismatch! Ignoring saved map.")
                    return False
                for x in range(self.width):
                    row = f.readline().strip().split(",")
                    for y in range(self.height):
                        self.walls[x][y] = int(row[y])
            print(f"Maze loaded from {filename}")
            return True
        except:
            print("No saved maze found. Starting fresh.")
            return False

    def print_maze(self):
        """Basic visualization of the maze walls to console."""
        for y in range(self.height - 1, -1, -1):
            row_str = ""
            for x in range(self.width):
                if self.has_wall(x, y, NORTH):
                    row_str += "+---"
                else:
                    row_str += "+   "
            print(row_str + "+")
            
            row_str = ""
            for x in range(self.width):
                if self.has_wall(x, y, WEST):
                    row_str += "| " + f"{self.distances[x][y]:02} "
                else:
                    row_str += "  " + f"{self.distances[x][y]:02} "
            if self.has_wall(self.width - 1, y, EAST):
                   row_str += "|"
            print(row_str)
            
        row_str = ""
        for x in range(self.width):
            row_str += "+---"
        print(row_str + "+")


if __name__ == "__main__":
    # Test our mapping logic!
    m = Maze()
    
    # Let's say we started at (0,0) facing NORTH, and we saw a wall on the Right and Left, but front was open.
    # Cell 0,0 usually has a wall behind it (SOUTH) physically, but our perimeter function added that.
    m.update_cell_walls(0, 0, Heading.NORTH, wall_left=True, wall_front=False, wall_right=True)
    m.print_maze()
