/**
 * @file sunray_fsm.hpp
 * @brief Sunray项目
 * @see
 *
 */

// Sunray_FSM指的是Sunray项目框架中的无人机控制状态机，然而由于我们希望Sunray项目的框架能够实现的，兼容或者说具有强的扩展性，
// 在实现的过程中，我们希望这个Sunray_FSM能够进行一定的抽象，因此SUnray_FSM与mavros进行解耦
// 也就说是Sunray_FSM只涉及到状态机的实现，不涉及到具体的无人机控制逻辑
// 为了达到这一点，我们得到这样的一些思路
// 1. 设计一个类用于检查无人机的状态，这样做的好处是将FSM与Mavros解耦
// 2. 设计一个类用于检查无人机的参数，这样做的好处是将FSM与Mavros解耦
// 设计一个类用于实现无人机的控制，这样做的好处是将FSM与Mavros解耦
// 这三点，全部交给controller来实现
