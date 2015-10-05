package kdtree

import (
	"container/heap"
	"math"
)

type Point interface {
	Coord(int) float64
	Dim() int
}

type Node struct {
	dim         int
	axis        float64
	value       Point
	left, right *Node
}

func Build(points []Point) *Node {
	return buildTree(points, 0)
}

func (node *Node) KNearestNeighbour(point Point, k int) []Point {

	ph := pointHeap{}
	node.kNNSearch(point, k, &ph)

	result := make([]Point, int(math.Min(float64(k), float64(len(ph)))))
	for i := 0; i < k; i++ {
		result[i] = heap.Pop(&ph).(pointHeapItem).P
	}

	return result
}

func buildTree(points []Point, depth int) *Node {
	var n Node

	switch len(points) {
	case 0:
		break

	case 1:
		n.value = points[0]
		break

	default:
		median := len(points) / 2
		n.dim = depth % points[0].Dim()
		qsort(points, n.dim)
		n.axis = points[median].Coord(n.dim)
		n.left = buildTree(points[:median], depth+1)
		n.right = buildTree(points[median+1:], depth+1)
		break
	}

	return &n
}

func (node *Node) kNNSearch(point Point, k int, result *pointHeap) {
	// Approximation factor for nearest neighbour search
	eps := 1.05

	if node.value != nil {
		squaredDistance := 0.0
		for i := 0; i < node.value.Dim(); i++ {
			squaredDistance += (node.value.Coord(i) - point.Coord(i)) * (node.value.Coord(i) - point.Coord(i))
		}
		heap.Push(result, pointHeapItem{node.value, squaredDistance})
		return
	}

	squaredDistanceToAxis := eps * (point.Coord(node.dim) - node.axis) * (point.Coord(node.dim) - node.axis)
	if point.Coord(node.dim) < node.axis {
		if node.left != nil {
			node.left.kNNSearch(point, k, result)
		}
		if node.right != nil && (result.Len() < k || squaredDistanceToAxis < result.Peek().Distance) {
			node.right.kNNSearch(point, k, result)
		}
	} else {
		if node.right != nil {
			node.right.kNNSearch(point, k, result)
		}
		if node.left != nil && (result.Len() < k || squaredDistanceToAxis < result.Peek().Distance) {
			node.left.kNNSearch(point, k, result)
		}
	}
}

type pointHeapItem struct {
	P        Point
	Distance float64
}

type pointHeap []pointHeapItem

func (ph pointHeap) Len() int { return len(ph) }

func (ph pointHeap) Less(i, j int) bool {
	return ph[i].Distance < ph[j].Distance
}

func (ph pointHeap) Swap(i, j int) {
	ph[i], ph[j] = ph[j], ph[i]
}

func (ph *pointHeap) Push(x interface{}) {
	item := x.(pointHeapItem)
	*ph = append(*ph, item)
}

func (ph *pointHeap) Pop() interface{} {
	old := *ph
	n := len(old)
	item := old[n-1]
	*ph = old[0 : n-1]
	return item
}

func (ph pointHeap) Peek() pointHeapItem {
	item := ph[len(ph)-1]
	return item
}

func qsort(points []Point, dim int) {

	if len(points) > 1 {
		pivot := len(points) - 1
		i := 0

		for j := 0; j < pivot; j++ {
			if points[j].Coord(dim) < points[pivot].Coord(dim) {
				points[i], points[j] = points[j], points[i]
				i += 1
			}
		}
		points[i], points[pivot] = points[pivot], points[i]

		qsort(points[:i], dim)
		qsort(points[i+1:], dim)
	}
}
