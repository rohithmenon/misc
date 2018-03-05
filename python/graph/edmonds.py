class Node(object):
    def __init__(self, node_id, parent=None):
        self.node_id = node_id
        self.parent = parent
	self.matched = None
	self.neighbours = set()

    def get_node_id(self):
        return self.node_id

    def set_matched(self, match):
        self.matched = match

    def set_parent(self, node):
        if not self.parent and node.get_parent() != self:
            self.parent = node

    def clear_traversal(self):
        self.parent = None

    def get_matched(self):
        return self.matched

    def get_parent(self):
        return self.parent

    def connect(self, node):
        if self != node and node not in self.neighbours:
	    self.neighbours.add(node)

    def replace_neighbour(self, node1, node2):
        if node1 in self.neighbours:
	    self.neighbours.remove(node1)
        connect(self, node2)
        if self.parent == node1:
            self.parent = node2

    def get_neighbours(self):
        return self.neighbours

    def contract(self):
        pass

    def expand(self):
        pass

    def path(self, from_node, to_node):
        return [self]

    def num_children(self):
        return 1

    def __repr__(self):
        m = self.matched.get_node_id() if self.matched else None
        return "{}".format(self.node_id)

class SuperNode(Node):
    def __init__(self, node_id, children, cycle_root):
        super(SuperNode, self).__init__(node_id, parent=cycle_root.get_parent())
	self.children = children
        self.cycle_root = cycle_root
        self.reverse_children = list(self.children)
        self.reverse_children.reverse()
        self.contracted = False

    def contract(self):
        if self.contracted:
            return
	for child in self.children:
	    neighbours = child.neighbours
	    for neighbour in list(neighbours):
	        if neighbour not in self.children:
                    neighbour.replace_neighbour(child, self)
                    if child.matched == neighbour:
                        match(self, neighbour)
        self.contracted = True

    def expand(self):
        if not self.contracted:
           return
        for child in self.children:
	    neighbours = child.neighbours
	    for neighbour in list(neighbours):
	        if neighbour not in self.children:
                    neighbour.replace_neighbour(self, child)
                    if child.matched == neighbour:
                        match(child, neighbour)
        self.contracted = False

    def find_path_in_cycle(self, cycle):
        path = []
        idx = cycle.index(start_node)
        while True:
            path.append(cycle[idx])
            if (to_node in cycle[idx].get_neighbours() or not cycle[idx].get_parent()) \
                    and len(path) % 2 == 1:
                break
            idx = (idx + 1) % len(cycle)
        return path

    def path(self, from_node, to_node):
        candidates = from_node.get_neighbours() & set(self.children)
        for start_node in candidates:
            path = self.find_path_in_cycle(self.children)
            if len(path) % 2 == 1 and (to_node and to_node in path[-1].get_neighbours()) \
                    or (not to_node and not path[-1].get_parent()):
                return path
            path = self.find_path_in_cycle(self.reverse_children)
            if len(path) % 2 == 1 and (to_node and to_node in path[-1].get_neighbours()) \
                    or (not to_node and not path[-1].get_parent()):
                return path
        raise Exception("No valid path found")

    def num_children(self):
        return len(self.children)

    def __repr__(self):
        return "{}:{}".format(
                self.node_id,
                [n.get_node_id() for n in self.children])

def connect(n1, n2):
    n1.connect(n2)
    n2.connect(n1)

def match(n1, n2):
    n1.set_matched(n2)
    n2.set_matched(n1)

def mark_traversed(n1, n2):
    n2.set_parent(n1)

def find_path(node):
    path = []
    parent_node = node
    while parent_node != None:
        path.append(parent_node)
        parent_node = parent_node.get_parent()
    return path

def find_cycle(node_a, node_b):
    path_a = find_path(node_a)
    path_b = find_path(node_b)
    path_a.reverse()
    path_b.reverse()
    min_path_len = min(len(path_a), len(path_b))
    idx = 0
    while idx < min_path_len and (path_a[idx] == path_b[idx]):
        idx += 1
    cycle_b = path_b[idx:]
    cycle_a = path_a[idx-1:]
    cycle_a.reverse()
    return cycle_a + cycle_b, path_a[idx-1]

def clear_traversal(graph):
    for node in graph:
        node.clear_traversal()

def expand_path(path, depth=0):
    node_to_expand = None
    for part in path:
        if part.num_children() > 1:
            if node_to_expand:
                node_to_expand = max(node_to_expand, part, key=lambda n: n.get_node_id())
            else:
                node_to_expand = part

    if not node_to_expand:
        return path

    expanded_path = [path[0]]
    for i in range(1, len(path)):
        if path[i] == node_to_expand:
            path[i].expand()
            if i + 1 < len(path):
                expanded_path = expanded_path + path[i].path(expanded_path[-1], path[i + 1])
            else:
                expanded_path = expanded_path + path[i].path(expanded_path[-1], None)
        else:
            expanded_path = expanded_path + [path[i]]
    return expand_path(expanded_path, depth + 1)

def max_matching(graph):
    queue = []
    free_nodes = set(graph)
    sn_cnt = 0
    while free_nodes:
        clear_traversal(graph)
        free_node = free_nodes.pop()
        queue.append(free_node)
        level_tree = {free_node: 0}
        all_nodes = set(graph)
        while queue:
            node = queue.pop(0)
            for child in node.get_neighbours():
                if child not in level_tree and child.get_matched():
                    mate = child.get_matched()
                    level_tree[child] = 1
                    mark_traversed(node, child)
                    if mate not in level_tree:
                        mark_traversed(child, mate)
                        level_tree[mate] = 0
                        queue.append(mate)
                elif child in level_tree and level_tree[child] == 1:
                    continue
                elif child in level_tree and level_tree[child] == 0:
                    # Find cycle
                    cycle, cycle_root = find_cycle(node, child)
                    cycle_parent = cycle_root.get_parent()
                    super_node_id = 'sn_{:0>3d}'.format(sn_cnt)
                    sn_cnt += 1
                    super_node = SuperNode(super_node_id, cycle, cycle_root)
                    all_nodes.add(super_node)
                    super_node.contract()
                    super_node_added = False
                    for i in range(len(queue)):
                        if queue[i] in cycle:
                            queue[i] = super_node
                            super_node_added = True
                            break
                    if not super_node_added:
                        queue.append(super_node)

                    for cycle_node in cycle:
                        if cycle_node in queue:
                            queue.remove(cycle_node)
                    if not super_node_added:
                        queue.append(super_node)

                    level_tree[super_node] = 0
                    break
                elif child in free_nodes:
                    mark_traversed(node, child)
                    path = []
                    next_node = child
                    while next_node != None:
                        path.append(next_node)
                        next_node = next_node.get_parent()
                    path = expand_path(path)

                    for i in range(0, len(path), 2):
                        match(path[i], path[i+1])
                    for p in path:
                        if p in free_nodes:
                            free_nodes.remove(p)
                    for node in all_nodes:
                        node.expand()
                    queue = []
		    break
    max_matches = 0
    for node in graph:
        if node.get_matched():
            max_matches += 1
    return max_matches

def g1():
    n0 = Node(0)
    n1 = Node(1)
    n2 = Node(2)
    n3 = Node(3)
    n4 = Node(4)
    n5 = Node(5)

    connect(n0, n3)
    connect(n0, n4)
    connect(n0, n5)
    connect(n1, n2)
    connect(n1, n4)
    connect(n1, n5)
    connect(n2, n5)
    connect(n3, n4)

    return set([n0, n1, n2, n3, n4, n5])

def g2():
    n0 = Node(0)
    n1 = Node(1)
    n2 = Node(2)
    n3 = Node(3)
    n4 = Node(4)
    n5 = Node(5)

    connect(n0, n1)
    connect(n0, n2)
    connect(n0, n4)
    connect(n0, n5)
    connect(n1, n2)
    connect(n1, n3)
    connect(n2, n3)
    connect(n2, n4)
    return set([n0, n1, n2, n3, n4, n5])

if __name__ == "__main__":
    print max_matching(g1())
    print max_matching(g2())

