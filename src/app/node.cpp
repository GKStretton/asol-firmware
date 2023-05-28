#include "node.h"

machine_Node VialNumberToInsideNode(int number) {
	machine_Node n = (machine_Node) (number * 10 + 5);
	if (n < machine_Node_MIN_VIAL_INSIDE || n > machine_Node_MAX_VIAL_INSIDE) return machine_Node_UNDEFINED;
	return n;
}