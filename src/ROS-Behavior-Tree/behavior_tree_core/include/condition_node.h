/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef CONDITION_NODE_H
#define CONDITION_NODE_H

#include <leaf_node.h>
#include <string>

namespace BT
{
class ConditionNode : public LeafNode
{
public:
    // Constructor
    explicit ConditionNode(std::string name);
    ~ConditionNode();

    // The method that is going to be executed by the thread
    virtual BT::ReturnStatus Tick() = 0;

    // The method used to interrupt the execution of the node
    void Halt();

    // Methods used to access the node state without the
    // conditional waiting (only mutual access)
    bool WriteState(ReturnStatus new_state);
    int DrawType();
};
}  // namespace BT

#endif  // CONDITION_NODE_H
