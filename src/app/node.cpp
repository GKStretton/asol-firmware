#include "node.h"

Node VialNumberToInsideNode(int number) {
	Node n = (Node) (number * 10 + 5);
	if (n < MIN_VIAL_INSIDE || n > MAX_VIAL_INSIDE) return UNDEFINED;
	return n;
}