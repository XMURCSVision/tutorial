class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __lf__(self, other):
        return self.f < other.f
    
def astar(map, start, end):
    open_list = []
    closed_list = []
    start_node = Node(start[0], start[1])
    end_node = Node(end[0], end[1])
    open_list.append(start_node)
    while open_list:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        open_list.pop(current_index)
        closed_list.append(current_node)
        if current_node == end_node:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.x + new_position[0], current_node.y + new_position[1])
            if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map) - 1]) - 1) or node_position[1] < 0:
                continue
            if map[node_position[0]][node_position[1]] != 0:
                continue
            new_node = Node(node_position[0], node_position[1], current_node)
            children.append(new_node)
        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            child.g = current_node.g + 1
            child.h = ((child.x - end_node.x) ** 2) + ((child.y - end_node.y) ** 2)
            child.f = child.g + child.h
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)

map_ =  [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 1],
        [1, 0, 0, 0, 0, 0]
        ]
start = (0, 0)
end = (4, 5)
path = astar(map_, start, end)
print(path)