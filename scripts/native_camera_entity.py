#!/usr/bin/env python3
"""Own a camera entity lifecycle through the world simulation-v1 provider."""
import argparse
import json
import math
import re
import sys
if sys.version_info < (3, 8):
    raise RuntimeError("the selected camera interpreter must provide Python >= 3.8 and the formal XRPC wheel")

import rospy
from tf.transformations import quaternion_from_euler
from xgc2_xrpc import Client, Runtime, ServiceRef
from camera_peer import CameraPeer


def bind_simulation_service(service_ref_json, target_id, runtime):
    reference = ServiceRef.from_dict(json.loads(service_ref_json)).validate()
    if reference.service != "xgc2.simulation" or reference.api_version != "v1" or reference.profile != "http.v1":
        raise ValueError("an explicit simulation-v1 HTTP ServiceRef is required")
    if reference.target_id != target_id:
        raise ValueError("simulation ServiceRef target does not match the explicit target grant")
    return Client.from_service(reference, runtime=runtime, local_target=target_id), reference


class NativeCameraEntity:
    def __init__(self, service_ref_json, target_id, model_name, document, position, orientation):
        if not re.fullmatch(r"[A-Za-z0-9._-]{1,128}", model_name):
            raise ValueError("camera model name must be a bounded entity identifier")
        if not isinstance(document, str) or not document or len(document.encode()) > 65536:
            raise ValueError("camera definition must be a nonempty bounded URDF document")
        if not all(math.isfinite(value) for value in position + orientation):
            raise ValueError("camera pose must contain finite values")
        reference = ServiceRef.from_dict(json.loads(service_ref_json)).validate()
        if reference.service != "xgc2.simulation" or reference.api_version != "v1" or reference.profile != "http.v1" or reference.target_id != target_id:
            raise ValueError("an explicit local simulation-v1 HTTP ServiceRef is required")
        self._entity_ref = None
        self._reference = None
        self._operation = None
        self._submitted_instance = None
        self._entity = {"id": model_name, "role": "sensor", "asset": {"id": model_name + "-definition",
                        "realization": {"media_type": "application/urdf+xml", "content": document}},
                        "pose": {"position": position, "orientation": orientation}}
        self._peer = CameraPeer(reference.endpoint.address, target_id, "xgc2.simulation", self._create)

    def _create(self, client, description, stop):
        reference = ServiceRef.from_dict(description.get("service_ref", description.get("facts", {}).get("service_ref")))
        if "application/urdf+xml" not in description.get("artifact_types", description.get("facts", {}).get("artifact_types", [])):
            raise RuntimeError("simulation provider does not support the authored camera URDF realization")
        if self._reference != reference:
            self._entity_ref, self._operation, self._submitted_instance = None, None, None
            self._reference = reference
        if self._entity_ref is not None:
            return
        if self._operation is None:
            if self._submitted_instance == reference.instance_id:
                # A lost creation reply is reconciled by identity and authored
                # asset; it never causes a second POST in the same instance.
                result = client.json("/v1/entities/" + self._entity["id"], method="GET", timeout=3)
                entity = next(row for row in result["entities"] if row["ref"]["id"] == self._entity["id"])
                if entity.get("specification", {}).get("asset") != self._entity["asset"]:
                    raise RuntimeError("camera entity identity belongs to another authored asset")
                if entity.get("lifecycle") != "ready":
                    raise RuntimeError("native camera realization is not ready")
                self._entity_ref = entity["ref"]
                return
            from xgc2_xrpc import TransportError
            self._submitted_instance = reference.instance_id
            try:
                self._operation = client.json("/v1/entities", {"operation_timeout_ms": 10000, "entity": self._entity}, timeout=3)
            except TransportError as error:
                if error.disposition == "not_sent":
                    self._submitted_instance = None
                raise
        if not stop.is_set():
            completed = client.json("/v1/operations/" + self._operation["id"] + "/wait", {}, timeout=12)
            if completed.get("state") != "succeeded":
                raise RuntimeError("native camera creation failed: " + str(completed.get("error", completed)))
            entities = completed["result"]["entities"]
            self._entity_ref = next(entity["ref"] for entity in entities if entity["ref"]["id"] == self._entity["id"])

    def close(self):
        self._peer.close()
        if self._entity_ref is None:
            return
        runtime = Runtime(blocking_workers=1, max_calls=2, max_connections=2)
        client = Client.from_service(self._reference, runtime=runtime, local_target=self._reference.target_id)
        try:
            reference, self._entity_ref = self._entity_ref, None
            operation = client.json("/v1/entities/" + reference["id"],
                {"generation": reference["generation"], "operation_timeout_ms": 5000}, method="DELETE", timeout=3)
            completed = client.json("/v1/operations/" + operation["id"] + "/wait", {}, timeout=6)
            if completed.get("state") != "succeeded":
                raise RuntimeError("native camera deletion failed: " + str(completed.get("error", completed)))
        finally:
            client.close()
            runtime.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-service-ref-json", required=True)
    parser.add_argument("--target-id", required=True)
    parser.add_argument("--model-name", required=True)
    parser.add_argument("--description-param", required=True,
                        help="explicit authored bootstrap document; also published for user ROS algorithms")
    for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
        parser.add_argument("--" + axis, type=float, required=True)
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    rospy.init_node("native_camera_entity")
    entity = NativeCameraEntity(args.simulation_service_ref_json, args.target_id, args.model_name,
        rospy.get_param(args.description_param), [args.x, args.y, args.z],
        [float(value) for value in quaternion_from_euler(args.roll, args.pitch, args.yaw)])
    try:
        rospy.spin()
    finally:
        entity.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
