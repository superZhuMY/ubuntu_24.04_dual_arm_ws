"""Expose the next workflow step to the task executor."""

from uuid import uuid4

import rclpy
from double_arm_harvest_interfaces.srv import GetWorkflowStep
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .leaf_remove import leaf_remove
from .step import Step
from .tomato_pick import tomato_pick


def workflow_steps(request: GetWorkflowStep.Request) -> tuple[Step, ...]:
    if request.task_type == 'tomato_pick':
        return tomato_pick()
    if request.task_type == 'leaf_remove':
        return leaf_remove(
            execute_left=request.execute_left,
            execute_right=request.execute_right,
            left_role=request.left_role,
            right_role=request.right_role,
            left_retreat_delay_sec=request.left_retreat_delay_sec,
            right_retreat_delay_sec=request.right_retreat_delay_sec,
        )
    raise ValueError(f'unsupported task type {request.task_type!r}')


class WorkflowWorker(Node):
    def __init__(self):
        super().__init__('harvest_workflow_worker')
        self._worker_id = uuid4().hex
        self._service = self.create_service(
            GetWorkflowStep, 'get_workflow_step', self._get_step)

    def _get_step(
        self, request: GetWorkflowStep.Request, response: GetWorkflowStep.Response
    ) -> GetWorkflowStep.Response:
        try:
            steps = workflow_steps(request)
            index = request.completed_steps
            response.worker_id = self._worker_id
            if index == len(steps):
                response.skill = GetWorkflowStep.Response.DONE
                response.progress = 1.0
                return response
            if index > len(steps):
                raise ValueError(f'completed_steps={index} exceeds workflow length {len(steps)}')

            step = steps[index]
            response.skill = getattr(GetWorkflowStep.Response, step.skill.name)
            response.stage = step.stage
            response.execute_left = request.execute_left and step.execute_left
            response.execute_right = request.execute_right and step.execute_right
            response.use_j3_standoff = step.use_j3_standoff
            response.left_delay_sec = step.left_delay_sec if response.execute_left else 0.0
            response.right_delay_sec = step.right_delay_sec if response.execute_right else 0.0
            response.progress = step.progress
            return response
        except Exception as error:
            message = f'{type(error).__name__}: {error}'
            self.get_logger().error(f'Workflow request for task {request.task_id!r} failed: {message}')
            return GetWorkflowStep.Response(worker_id=self._worker_id, error=message)


def main():
    rclpy.init()
    node = WorkflowWorker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
