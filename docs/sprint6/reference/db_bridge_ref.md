## ROS Interactor Bridge
```py
"""
DB_Interactor - Private Database Access Layer for DB_Bridge

CRITICAL: This module is PRIVATE to DB_Bridge and should NEVER be imported
by other packages. All database access MUST go through DB_Bridge ROS 2 services.

This class merges DatabaseHandler and CompositionHelper functionality into a
single, thread-safe database access layer.

Architecture Version: Phase 1 (November 2025)
Location: ros_ws/src/cetibar_core/db_bridge/db_bridge/_db_interactor.py (PRIVATE)
"""

import os
import sys
import json
from typing import Optional, List, Dict, Any, Tuple, Set
from threading import RLock

# Import DatabaseHandler and DatabaseObjects from ros_assets
# Path: cetibar_team/ros_ws/src/db_bridge/db_bridge/_db_interactor.py (current file)
# Target: cetibar_team/ros_assets/database/src/ (4 steps up, then into ros_assets)
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
DB_SRC_PATH = os.path.join(WORKSPACE_ROOT, 'ros_assets', 'database', 'src')

if DB_SRC_PATH not in sys.path:
    sys.path.insert(0, DB_SRC_PATH)

try:
    from DatabaseHandler import DatabaseHandler
    from DatabaseObjects import (
        DBObj_Action,
        DBObj_Activity,
        DBObj_ActivityActionRel,
        DBObj_ActivityActionEdge,
        DBObj_Task,
        DBObj_TaskActivityRel,
        DBObj_Robot,
        DBObj_ActionType,
        DBObj_GripperType,
        DBObj_ActionDataPoint,
        DBObj_ActionDataPointCoords
    )
    DB_IMPORTS_AVAILABLE = True
except ImportError as e:
    print(f"[DB_Interactor] Failed to import database modules: {e}")
    DB_IMPORTS_AVAILABLE = False


# ============================================================================
# Exception Classes
# ============================================================================

class DBInteractorError(Exception):
    """Base exception for DB_Interactor errors."""
    pass


class ActionNotFoundError(DBInteractorError):
    """Raised when an action_id cannot be found."""
    pass


class ActivityNotFoundError(DBInteractorError):
    """Raised when an activity_id cannot be found."""
    pass


class ValidationError(DBInteractorError):
    """Raised when validation fails.
    
    Attributes:
        validation_report: Dict containing validation errors and warnings
    """
    def __init__(self, validation_report: Dict[str, Any]):
        self.validation_report = validation_report
        error_messages = [f"{e['code']}: {e['message']}" for e in validation_report.get('errors', [])]
        super().__init__(f"Validation failed with {len(validation_report.get('errors', []))} error(s): {'; '.join(error_messages)}")


# ============================================================================
# DB_Interactor Class
# ============================================================================

class DB_Interactor:
    """
    Database access layer for DB_Bridge (PRIVATE).
    
    IMPORTANT: This class is PRIVATE to DB_Bridge and should NEVER be
    passed as a parameter or imported in other modules. All database
    access MUST go through DB_Bridge ROS 2 services.
    
    Responsibilities:
    - Action/Activity/Task CRUD operations
    - Waypoint loading and saving
    - Activity composition (nodes + edges)
    - Activity validation
    - Dependency checking
    - Transaction management
    - Thread safety
    
    NOT Responsible For:
    - ROS 2 communication (no Action/Service clients)
    - Routing logic (that's PerformerHelper's job)
    - Robot control (that's adapters' job)
    
    Architecture:
    - Imports DatabaseHandler from ros_assets (existing, tested code)
    - Merges CompositionHelper functionality (copied methods)
    - Thread-safe via Lock on database operations
    """
    
    def __init__(self, db_path: Optional[str] = None):
        """
        Initialize DB_Interactor with DatabaseHandler.
        
        Args:
            db_path: Optional database path (default: uses ros_assets default)
        
        Raises:
            ImportError: If database modules not available
        """
        if not DB_IMPORTS_AVAILABLE:
            raise ImportError("Database modules not available. Check ros_assets/database/src/ path.")
        
        self.db_path = db_path
        self._lock = RLock()  # Reentrant lock for nested locking (thread safety)
        self._dbh = None  # Lazy initialization
    
    def _get_dbh(self):
        """Get DatabaseHandler context manager (lazy initialization, thread-safe).
        
        Returns a context manager that yields a DatabaseHandler instance.
        Each call creates a fresh connection to avoid cursor issues.
        """
        # Return context manager for with-statement
        return DatabaseHandler(db_path=self.db_path, autoCommit=True)
    
    def close(self):
        """Close database connection (called by DB_Bridge on cleanup)."""
        with self._lock:
            if self._dbh is not None:
                # DatabaseHandler closes connection in __del__
                self._dbh = None

    @staticmethod
    def _safe_count(dbh: "DatabaseHandler", table_name: str) -> int:
        try:
            row = dbh.execute(f"SELECT COUNT(*) FROM {table_name}").fetchone()
        except Exception:
            return 0
        if row is None:
            return 0
        return int(row[0] or 0)

    @staticmethod
    def _robot_row_to_dict(robot_row: Any) -> Dict[str, Any]:
        robot = dict(robot_row)
        return {
            'rowid': int(robot.get('rowid', 0) or 0),
            'robot_id': robot.get('robot_id') or '',
            'ipv4': robot.get('ipv4') or '',
            'descshort': robot.get('descshort') or '',
            'desclong': robot.get('desclong') or '',
        }

    def initialize_database(self) -> None:
        """Create the schema for a fresh CeTIBAR database and seed lookup rows."""
        with self._lock:
            with self._get_dbh() as dbh:
                DBObj_Robot.create_tables(dbh)
                DBObj_ActionType.create_tables(dbh)
                DBObj_GripperType.create_tables(dbh)
                DBObj_Task.create_tables(dbh)
                DBObj_Activity.create_tables(dbh)
                DBObj_Action.create_tables(dbh)
                DBObj_ActionDataPoint.create_tables(dbh)
                DBObj_ActionDataPointCoords.create_tables(dbh)
                DBObj_ActivityActionRel.create_tables(dbh)
                DBObj_ActivityActionEdge.create_tables(dbh)
                DBObj_TaskActivityRel.create_tables(dbh)

                DBObj_Robot.default_insert(dbh)
                DBObj_ActionType.default_insert(dbh)
                DBObj_GripperType.default_insert(dbh)

    def get_database_counts(self) -> Dict[str, int]:
        """Return lightweight entity counts for database-management UIs."""
        with self._lock:
            with self._get_dbh() as dbh:
                return {
                    'action_count': self._safe_count(dbh, DBObj_Action.tablename),
                    'activity_count': self._safe_count(dbh, DBObj_Activity.tablename),
                    'task_count': self._safe_count(dbh, DBObj_Task.tablename),
                    'robot_count': self._safe_count(dbh, DBObj_Robot.tablename),
                }

    def query_robot_entries(self) -> List[Dict[str, Any]]:
        """Return all robot lookup rows from the active database."""
        with self._lock:
            with self._get_dbh() as dbh:
                rows = DBObj_Robot.fetch_all(dbh)
                return [self._robot_row_to_dict(row) for row in rows]

    def update_robot_entry(
        self,
        rowid: int,
        *,
        robot_id: str,
        ipv4: Optional[str] = None,
        descshort: str = '',
        desclong: str = '',
    ) -> Dict[str, Any]:
        """Update one robot lookup entry in place."""
        normalized_robot_id = (robot_id or '').strip()
        if not normalized_robot_id:
            raise DBInteractorError('robot_id is required')

        normalized_ipv4 = (ipv4 or '').strip() or None

        with self._lock:
            with self._get_dbh() as dbh:
                current_row = DBObj_Robot.fetch_one_by_row_id(dbh, rowid)
                if current_row is None:
                    raise DBInteractorError(f"Robot rowid '{rowid}' not found")

                existing_robot = DBObj_Robot.fetch_one(dbh, normalized_robot_id)
                if existing_robot is not None and int(existing_robot['rowid']) != int(rowid):
                    raise DBInteractorError(f"Robot '{normalized_robot_id}' already exists")

                if normalized_ipv4 is not None:
                    existing_ipv4 = DBObj_Robot.fetch_one_by_ipv4(dbh, normalized_ipv4)
                    if existing_ipv4 is not None and int(existing_ipv4['rowid']) != int(rowid):
                        raise DBInteractorError(f"IPv4 '{normalized_ipv4}' is already assigned")

                dbh.execute(
                    (
                        f"UPDATE {DBObj_Robot.tablename} "
                        "SET robot_id = ?, ipv4 = ?, descshort = ?, desclong = ? "
                        "WHERE rowid = ?"
                    ),
                    (
                        normalized_robot_id,
                        normalized_ipv4,
                        descshort or '',
                        desclong or '',
                        int(rowid),
                    ),
                )

                updated_row = DBObj_Robot.fetch_one_by_row_id(dbh, rowid)
                if updated_row is None:
                    raise DBInteractorError(f"Robot rowid '{rowid}' not found after update")
                return self._robot_row_to_dict(updated_row)
    
    # ========================================================================
    # Action Operations (Task 3)
    # ========================================================================
    
    def load_action(self, action_id: str) -> Dict[str, Any]:
        """
        Load action metadata by action_id.
        
        Args:
            action_id: Action identifier
            
        Returns:
            Dict with action metadata:
            {
                "rowid": int,
                "action_id": str,
                "actiontype_id": str,
                "robot_id": str,
                "duration_ms": int,
                "waypoint_count": int,
                "count_dof": int,
                "metadata": dict
            }
            
        Raises:
            ActionNotFoundError: If action_id not found
        """
        with self._lock:
            with self._get_dbh() as dbh:
                action_row = DBObj_Action.fetch_one(dbh, action_id)
                if action_row is None:
                    raise ActionNotFoundError(f"Action '{action_id}' not found")
                
                # Convert Row to dict
                action = dict(action_row)
                
                # Resolve robot_id and actiontype_id from foreign keys using DBObj helpers
                robot_row = DBObj_Robot.fetch_one_by_row_id(dbh, action["robot_rowid"])
                actiontype_row = DBObj_ActionType.fetch_one_by_row_id(dbh, action["actiontype_rowid"])
                
                # Parse metadata JSON if present (check metadata_json field)
                metadata = {}
                if "metadata_json" in action and action["metadata_json"]:
                    try:
                        metadata = json.loads(action["metadata_json"])
                    except json.JSONDecodeError:
                        pass
                
                return {
                    "rowid": action["rowid"],
                    "action_id": action["action_id"],
                    "actiontype_id": actiontype_row["actiontype_id"] if actiontype_row else None,
                    "robot_id": robot_row["robot_id"] if robot_row else None,
                    "duration_ms": action.get("duration_ms", 0),
                    "waypoint_count": action.get("waypoint_count", 0),
                    "count_dof": action.get("count_dof", 0),
                    "metadata": metadata
                }
    
    def load_waypoints(self, action_id: str) -> List[Dict[str, Any]]:
        """
        Load all waypoints for an action.
        
        Args:
            action_id: Action identifier
            
        Returns:
            List of waypoint dicts:
            [
                {
                    "seq_no": int,
                    "t_ms": int,
                    "generalized_coords": List[float],  # radians/meters
                    "gripper_width": float,             # meters (optional)
                    "gripper_force": float,             # newtons (optional)
                    "gripper_load": float               # kg (optional)
                },
                ...
            ]
            
        Raises:
            ActionNotFoundError: If action_id not found
        """
        with self._lock:
            with self._get_dbh() as dbh:
                # Get action rowid
                action_row = DBObj_Action.fetch_one(dbh, action_id)
                if action_row is None:
                    raise ActionNotFoundError(f"Action '{action_id}' not found")
                
                action_rowid = action_row["rowid"]
                
                # Fetch waypoints (DBObj_ActionDataPoint.fetch_all_by_action_row_id merges coords)
                waypoints = DBObj_ActionDataPoint.fetch_all_by_action_row_id(dbh, action_rowid)
                
                # Convert to standard format
                result = []
                for wp in waypoints:
                    wp_dict = dict(wp)
                    result.append({
                        "seq_no": wp_dict["seq_no"],
                        "t_ms": wp_dict["t_ms"],
                        "generalized_coords": wp_dict.get("generalized_coords", []),
                        "gripper_width": wp_dict.get("gripper_width"),
                        "gripper_force": wp_dict.get("gripper_force"),
                        "gripper_load": wp_dict.get("gripper_load")
                    })
                
                return result

    def get_action_id_by_rowid(self, rowid: int) -> str:
        """Resolve an action_id from a persisted SQLite rowid."""
        with self._lock:
            with self._get_dbh() as dbh:
                action_row = DBObj_Action.fetch_one_by_row_id(dbh, rowid)
                if action_row is None:
                    raise ActionNotFoundError(f"Action rowid '{rowid}' not found")
                return action_row["action_id"]

    def get_activity_id_by_rowid(self, rowid: int) -> str:
        """Resolve an activity_id from a persisted SQLite rowid."""
        with self._lock:
            with self._get_dbh() as dbh:
                activity_row = DBObj_Activity.fetch_one_by_row_id(dbh, rowid)
                if activity_row is None:
                    raise ActivityNotFoundError(f"Activity rowid '{rowid}' not found")
                return activity_row["activity_id"]
    
    def save_action(self, action_id: str, actiontype_id: str, robot_id: str, 
                   waypoints: List[Dict[str, Any]], metadata: Optional[Dict] = None, 
                   overwrite: bool = False) -> int:
        """
        Save action with waypoints to database.
        
        Args:
            action_id: Unique action identifier
            actiontype_id: Action type (Trajectory, Gripper, MoveToConfig, PortalPose)
            robot_id: Robot identifier
            waypoints: List of waypoint dicts (see load_waypoints for structure)
            metadata: Optional action metadata (will be JSON-serialized)
            overwrite: If True, delete existing action before saving
            
        Returns:
            Action rowid
            
        Raises:
            DBInteractorError: If robot_id or actiontype_id not found, or action_id exists and overwrite=False
        """
        with self._lock:
            with self._get_dbh() as dbh:
                # Check if action exists
                existing_action = DBObj_Action.fetch_one(dbh, action_id)
                if existing_action is not None:
                    if not overwrite:
                        raise DBInteractorError(f"Action '{action_id}' already exists. Set overwrite=True to replace.")
                    # Delete existing action (cascades to waypoints)
                    DBObj_Action.delete_one(dbh, action_id)
                
                # Validate robot and action type exist
                robot_row = DBObj_Robot.fetch_one(dbh, robot_id)
                if robot_row is None:
                    raise DBInteractorError(f"Robot '{robot_id}' not found")
                
                actiontype_row = DBObj_ActionType.fetch_one(dbh, actiontype_id)
                if actiontype_row is None:
                    raise DBInteractorError(f"ActionType '{actiontype_id}' not found")
                
                # Calculate metadata fields for DBObj_Action
                duration_ms = waypoints[-1]["t_ms"] if waypoints else 0
                waypoint_count = len(waypoints)
                count_dof = len(waypoints[0]["generalized_coords"]) if waypoints and waypoints[0].get("generalized_coords") else 0
                
                # Prepare metadata dict (will be stored as metadata_json internally)
                action_metadata = metadata.copy() if metadata else {}
                action_metadata.update({
                    'duration_ms': duration_ms,
                    'waypoint_count': waypoint_count,
                    'count_dof': count_dof
                })
                
                # Insert action metadata (DBObj_Action.insert_one handles metadata internally)
                DBObj_Action.insert_one(
                    dbh,
                    action_id=action_id,
                    actiontype_id=actiontype_id,
                    robot_id=robot_id,
                    metadata=action_metadata
                )
                
                # Get action rowid for waypoint insertion
                action_row = DBObj_Action.fetch_one(dbh, action_id)
                action_rowid = action_row["rowid"]
                
                # Insert waypoints (automatically creates ActionDataPointCoords entries)
                if waypoints:
                    DBObj_ActionDataPoint.insert_by_action_id(dbh, action_id, waypoints)
                
                return action_rowid

    def update_action(
        self,
        rowid: int,
        *,
        action_id: Optional[str] = None,
        robot_id: Optional[str] = None,
        actiontype_id: Optional[str] = None,
        metadata_updates: Optional[Dict[str, Any]] = None,
        expected_revision: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Update persisted action metadata in place.

        Supports optimistic concurrency via ``expected_revision`` stored in the
        action metadata JSON payload.
        """
        with self._lock:
            with self._get_dbh() as dbh:
                action_row = DBObj_Action.fetch_one_by_row_id(dbh, rowid)
                if action_row is None:
                    raise ActionNotFoundError(f"Action rowid '{rowid}' not found")

                current_action_id = action_row['action_id']
                new_action_id = action_id or current_action_id

                if new_action_id != current_action_id:
                    existing = DBObj_Action.fetch_one(dbh, new_action_id)
                    if existing is not None:
                        raise DBInteractorError(f"Action ID '{new_action_id}' already exists")

                metadata = {}
                if action_row['metadata_json']:
                    try:
                        metadata = json.loads(action_row['metadata_json'])
                    except json.JSONDecodeError:
                        metadata = {}

                try:
                    current_revision = int(metadata.get('revision', 0) or 0)
                except (TypeError, ValueError):
                    current_revision = 0

                if expected_revision is not None and expected_revision != current_revision:
                    raise DBInteractorError(
                        f"STALE_REVISION expected={expected_revision} current={current_revision}"
                    )

                update_fields: List[str] = []
                params: List[Any] = []

                if new_action_id != current_action_id:
                    update_fields.append('action_id = ?')
                    params.append(new_action_id)

                if robot_id is not None:
                    robot_row = DBObj_Robot.fetch_one(dbh, robot_id)
                    if robot_row is None:
                        raise DBInteractorError(f"Robot '{robot_id}' not found")
                    update_fields.append('robot_rowid = ?')
                    params.append(robot_row['rowid'])

                if actiontype_id is not None:
                    actiontype_row = DBObj_ActionType.fetch_one(dbh, actiontype_id)
                    if actiontype_row is None:
                        raise DBInteractorError(f"ActionType '{actiontype_id}' not found")
                    update_fields.append('actiontype_rowid = ?')
                    params.append(actiontype_row['rowid'])

                if update_fields:
                    params.append(rowid)
                    dbh.execute(
                        f"UPDATE {DBObj_Action.tablename} SET {', '.join(update_fields)} WHERE rowid = ?",
                        tuple(params),
                    )

                merged_metadata = metadata.copy()
                if metadata_updates:
                    merged_metadata.update(metadata_updates)
                merged_metadata['revision'] = current_revision + 1
                DBObj_Action._update_metadata(dbh, new_action_id, merged_metadata)

        return self.load_action(new_action_id)
    
    def delete_action(self, action_id: str, force: bool = False) -> bool:
        """
        Delete action with dependency checking.
        
        Args:
            action_id: Action identifier to delete
            force: If True, delete even if used in activities (does NOT cascade to activities,
                   only removes action from database - activities will have invalid references)
            
        Returns:
            True if deleted, False if dependencies exist and force=False
            
        Raises:
            ActionNotFoundError: If action_id not found
            
        Note:
            When force=True, activities using this action will have invalid references.
        """
        with self._lock:
            with self._get_dbh() as dbh:
                action_row = DBObj_Action.fetch_one(dbh, action_id)
                if action_row is None:
                    raise ActionNotFoundError(f"Action '{action_id}' not found")
                
                # Check dependencies using DatabaseObjects method
                dependency_count = DBObj_Action.count_usages(dbh, action_id)
                
                if dependency_count > 0 and not force:
                    return False  # Dependencies exist
                
                # Delete action (cascades to waypoints via FK, but NOT to activities)
                DBObj_Action.delete_one(dbh, action_id)
                return True
    
    def query_actions(self, robot_id: Optional[str] = None, 
                     actiontype_id: Optional[str] = None,
                     search: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Query actions with optional filters.
        
        Args:
            robot_id: Filter by robot (if specified)
            actiontype_id: Filter by action type (if specified)
            search: Filter by action_id substring (case-insensitive, if specified)
        
        Returns:
            List of action metadata dicts (see load_action for structure)
        """
        with self._lock:
            with self._get_dbh() as dbh:
                all_actions = DBObj_Action.fetch_all(dbh)
                
                result = []
                for action_row in all_actions:
                    action_id = action_row["action_id"]
                    action = self.load_action(action_id)
                    
                    # Apply filters
                    if robot_id and action.get("robot_id") != robot_id:
                        continue
                    if actiontype_id and action.get("actiontype_id") != actiontype_id:
                        continue
                    if search and search.lower() not in action_id.lower():
                        continue
                    result.append(action)
                
                return result
    
    # ========================================================================
    # Activity Operations (Task 4) - Merged from CompositionHelper
    # ========================================================================
    
    def load_activity(self, activity_id: str) -> Dict[str, Any]:
        """
        Load activity with full graph structure (nodes + edges).
        
        Args:
            activity_id: Activity identifier
            
        Returns:
            Dict with structure:
            {
                "activity_id": str,
                "activity_rowid": int,
                "nodes": List[Dict],  # [{action_no, action_id, sync_flag}]
                "edges": List[Tuple[int, int]]  # [(src_action_no, dst_action_no)]
            }
            
        Raises:
            ActivityNotFoundError: If activity_id not found
        """
        with self._lock:
            with self._get_dbh() as dbh:
                # Get activity metadata
                activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
                if activity_row is None:
                    raise ActivityNotFoundError(f"Activity '{activity_id}' not found")
                
                activity_rowid = activity_row["rowid"]
                
                # Load nodes (fetch_nodes returns list of tuples: (action_no, action_id, sync_flag))
                nodes_data = DBObj_ActivityActionRel.fetch_nodes(dbh, activity_id)
                nodes = []
                for action_no, action_id, sync_flag in nodes_data:
                    nodes.append({
                        "action_no": action_no,
                        "action_id": action_id,
                        "sync_flag": sync_flag
                    })
                
                # Load edges (fetch_by_activity returns list of tuples: (activity_id, src, dst))
                edges_data = DBObj_ActivityActionEdge.fetch_by_activity(dbh, activity_id)
                edges = [(src, dst) for _, src, dst in edges_data]
                
                return {
                    "activity_id": activity_id,
                    "activity_rowid": activity_rowid,
                    "nodes": nodes,
                    "edges": edges
                }
    
    def save_activity(self, activity_id: str, nodes: List[Dict[str, Any]], 
                     edges: List[Tuple[int, int]], validate: bool = True) -> None:
        """
        Save activity with validation (replace-write inside transaction).
        
        Args:
            activity_id: Activity identifier
            nodes: List of node dicts [{action_no, action_id, sync_flag}]
            edges: List of edge tuples [(src_action_no, dst_action_no)]
            validate: If True, validate structure before saving
            
        Raises:
            ValidationError: If validation fails
            DBInteractorError: If database operation fails
        """
        # Validate structure if requested
        if validate:
            report = self.validate_activity_structure(nodes, edges)
            if not report["success"]:
                raise ValidationError(report)
        
        with self._lock:
            with self._get_dbh() as dbh:
                # Begin transaction (Task 5: Transaction management)
                original_autocommit = dbh.autoCommit
                dbh.autoCommit = False
                
                try:
                    # Resolve or create activity
                    activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
                    if activity_row is None:
                        # Create new activity with default time
                        DBObj_Activity.insert_one(dbh, activity_id, activitytime=0)
                        activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
                    
                    activity_rowid = activity_row["rowid"]
                    
                    # Delete existing edges and nodes (in correct order due to FK constraints)
                    DBObj_ActivityActionEdge.delete_by_activity(dbh, activity_id)
                    DBObj_ActivityActionRel.delete_by_activity(dbh, activity_id)
                    
                    # Resolve action_ids → action_rowids
                    action_rowid_map = {}
                    for node in nodes:
                        action_row = DBObj_Action.fetch_one(dbh, node["action_id"])
                        if action_row is None:
                            raise DBInteractorError(f"Action '{node['action_id']}' not found")
                        action_rowid_map[node["action_id"]] = action_row["rowid"]
                    
                    # Insert new nodes
                    if nodes:
                        nodes_data = [
                            {
                                "action_id": node["action_id"],
                                "action_no": node["action_no"],
                                "sync_flag": node.get("sync_flag")
                            }
                            for node in nodes
                        ]
                        DBObj_ActivityActionRel.insert_nodes(dbh, activity_id, nodes_data)
                    
                    # Insert new edges (deduplicated)
                    if edges:
                        edges_list = list(set(edges))  # Remove duplicates
                        DBObj_ActivityActionEdge.insert_edges(dbh, activity_id, edges_list)
                    
                    # Update activity time (sum of action durations - simplified)
                    total_time = sum(
                        (DBObj_Action.fetch_one(dbh, node["action_id"])["duration_ms"] or 0)
                        for node in nodes
                    )
                    DBObj_Activity.update_time(dbh, activity_id, total_time)
                    
                    # Commit transaction
                    dbh.commit()
                    
                except Exception as e:
                    # Rollback on error (Task 5: Transaction rollback)
                    dbh.rollback()
                    raise DBInteractorError(f"Failed to save activity: {e}") from e
                
                finally:
                    # Restore original autocommit setting
                    dbh.autoCommit = original_autocommit
    
    def delete_activity(self, activity_id: str, force: bool = False) -> bool:
        """
        Delete activity with dependency checking.
        
        Args:
            activity_id: Activity identifier to delete
            force: If True, delete even if used in tasks (does NOT cascade to tasks)
            
        Returns:
            True if deleted, False if dependencies exist and force=False
            
        Raises:
            ActivityNotFoundError: If activity_id not found
        """
        with self._lock:
            with self._get_dbh() as dbh:
                activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
                if activity_row is None:
                    raise ActivityNotFoundError(f"Activity '{activity_id}' not found")
                
                # Check dependencies using DatabaseObjects method
                dependency_count = DBObj_Activity.count_usages(dbh, activity_id)
                
                if dependency_count > 0 and not force:
                    return False  # Dependencies exist
                
                # Delete activity (cascades to nodes and edges via FK)
                DBObj_Activity.delete_one(dbh, activity_id)
                return True
    
    def query_activities(self, search: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Query activities with optional search filter.
        
        Args:
            search: Filter by activity_id substring (case-insensitive, if specified)
        
        Returns:
            List of activity metadata dicts
        """
        with self._lock:
            with self._get_dbh() as dbh:
                all_activities = DBObj_Activity.fetch_all(dbh)
                
                result = []
                for activity_row in all_activities:
                    activity = dict(activity_row)
                    activity_id = activity["activity_id"]
                    
                    # Apply search filter
                    if search and search.lower() not in activity_id.lower():
                        continue

                    node_rows = DBObj_ActivityActionRel.fetch_nodes(dbh, activity_id)
                    edge_rows = DBObj_ActivityActionEdge.fetch_by_activity(dbh, activity_id)
                    action_ids = [action_id for _, action_id, _ in node_rows]
                    
                    result.append({
                        "rowid": activity["rowid"],
                        "activity_id": activity_id,
                        "activitytime": activity.get("activitytime", 0),
                        "node_count": len(node_rows),
                        "edge_count": len(edge_rows),
                        "action_ids": action_ids,
                    })
                
                return result
    
    def create_activity(self, activity_id: str) -> int:
        """
        Create empty activity shell (no nodes/edges) - Task 5.1b.
        
        Args:
            activity_id: Activity identifier
            
        Returns:
            Activity rowid
            
        Raises:
            DBInteractorError: If activity_id already exists
        """
        with self._lock:
            with self._get_dbh() as dbh:
                # Check if activity already exists
                existing = DBObj_Activity.fetch_one(dbh, activity_id)
                if existing is not None:
                    raise DBInteractorError(f"Activity '{activity_id}' already exists")
                
                # Create empty activity with default time
                DBObj_Activity.insert_one(dbh, activity_id, activitytime=0)
                
                # Return rowid
                activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
                return activity_row["rowid"]
    
    # ========================================================================
    # Task Operations (Task 5.1c + 5.1d) - Read-Only for Phase 1
    # ========================================================================
    
    def query_tasks(self, search: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Query tasks with optional search filter - READ-ONLY for Phase 1.
        
        Args:
            search: Filter by task_id substring (case-insensitive, if specified)
        
        Returns:
            List of task metadata dicts
            
        Note:
            Task write operations (create, save, delete) are deferred to Phase 2 Task Manager.
        """
        with self._lock:
            with self._get_dbh() as dbh:
                all_tasks = DBObj_Task.fetch_all(dbh)
                
                result = []
                for task_row in all_tasks:
                    task = dict(task_row)
                    task_id = task["task_id"]
                    
                    # Apply search filter
                    if search and search.lower() not in task_id.lower():
                        continue
                    
                    result.append({
                        "rowid": task["rowid"],
                        "task_id": task_id,
                        "tasktime": task.get("tasktime", 0)
                    })
                
                return result
    
    def load_task(self, task_id: str) -> Dict[str, Any]:
        """
        Load task composition (ordered activity_ids) - READ-ONLY for Phase 1.
        
        Args:
            task_id: Task identifier
            
        Returns:
            Dict with structure:
            {
                "task_id": str,
                "task_rowid": int,
                "activity_ids": List[str],  # Ordered list of activities
                "tasktime": int
            }
            
        Raises:
            ActivityNotFoundError: If task_id not found (reusing exception for consistency)
            
        Note:
            Task write operations are deferred to Phase 2 Task Manager.
        """
        with self._lock:
            with self._get_dbh() as dbh:
                task_row = DBObj_Task.fetch_one(dbh, task_id)
                if task_row is None:
                    raise ActivityNotFoundError(f"Task '{task_id}' not found")  # Reuse exception
                
                task_rowid = task_row["rowid"]
                
                # Load activity relationships (ordered by activity_no)
                rels = DBObj_TaskActivityRel.fetch_all_by_task_rowid(dbh, task_rowid)
                
                # Resolve activity_rowids → activity_ids
                activity_ids = []
                for rel in rels:
                    activity_rowid = rel["activity_rowid"] if isinstance(rel, dict) else rel["activity_rowid"]
                    activity = DBObj_Activity.fetch_one_by_row_id(dbh, activity_rowid)
                    if activity is not None:
                        activity_ids.append(activity["activity_id"])
                
                return {
                    "task_id": task_id,
                    "task_rowid": task_rowid,
                    "activity_ids": activity_ids,
                    "tasktime": task_row.get("tasktime", 0)
                }
    
    # ========================================================================
    # Validation (Task 4 + Task 5.1a) - Merged from CompositionHelper
    # ========================================================================
    
    def validate_activity_structure(self, nodes: List[Dict], edges: List[Tuple]) -> Dict[str, Any]:
        """
        Validate activity structure (6 checks) - PUBLIC for ValidateActivity service.
        
        Validation Checks:
        1. DUP_ACTION_NO: Duplicate action_no values
        2. UNKNOWN_ACTION: action_id not in database
        3. ORPHAN_NODE: Node without edges (invalid for multi-node)
        4. CYCLE: Cyclic dependency (must be DAG)
        5. SYNC_INCOMPLETE: Sync group lacks merge point (simplified check)
        6. ROBOT_PARALLEL_CONFLICT: Same robot in parallel branches (warning)
        
        Returns:
            Dict with structure:
            {
                "success": bool,
                "errors": List[Dict],    # [{code, message, details}]
                "warnings": List[Dict],  # [{code, message, details}]
                "topo_order": List[int]  # Topological order (if DAG valid)
            }
        """
        errors = []
        warnings = []
        topo_order = []
        
        # Check 1: Duplicate action_no
        action_nos = [node["action_no"] for node in nodes]
        if len(action_nos) != len(set(action_nos)):
            duplicates = [no for no in set(action_nos) if action_nos.count(no) > 1]
            errors.append({
                "code": "DUP_ACTION_NO",
                "message": f"Duplicate action_no values: {duplicates}",
                "details": {"duplicates": duplicates}
            })
        
        # Check 2: Unknown actions (with database lock already held by caller)
        for node in nodes:
            with self._get_dbh() as dbh:
                action_row = DBObj_Action.fetch_one(dbh, node["action_id"])
                if action_row is None:
                    errors.append({
                        "code": "UNKNOWN_ACTION",
                        "message": f"Unknown action_id: {node['action_id']}",
                        "details": {"action_id": node["action_id"]}
                    })
        
        # Check 3: Orphan nodes (if more than 1 node)
        if len(nodes) > 1:
            edge_set = set()
            for src, dst in edges:
                edge_set.add(src)
                edge_set.add(dst)
            
            orphans = [node["action_no"] for node in nodes if node["action_no"] not in edge_set]
            if orphans:
                errors.append({
                    "code": "ORPHAN_NODE",
                    "message": f"Nodes without edges: {orphans}",
                    "details": {"orphans": orphans}
                })
        
        # Check 4: Cycle detection (Kahn's algorithm)
        if edges and not errors:
            nodeset = {node["action_no"] for node in nodes}
            in_degree = {no: 0 for no in nodeset}
            adjacency = {no: [] for no in nodeset}
            
            for src, dst in edges:
                adjacency[src].append(dst)
                in_degree[dst] += 1
            
            queue = [no for no in nodeset if in_degree[no] == 0]
            topo_order = []
            
            while queue:
                queue.sort()  # Deterministic ordering
                current = queue.pop(0)
                topo_order.append(current)
                
                for neighbor in adjacency[current]:
                    in_degree[neighbor] -= 1
                    if in_degree[neighbor] == 0:
                        queue.append(neighbor)
            
            if len(topo_order) != len(nodeset):
                errors.append({
                    "code": "CYCLE",
                    "message": "Cycle detected in activity graph",
                    "details": {"processed": len(topo_order), "total": len(nodeset)}
                })
                topo_order = []
        
        # Note: Checks 5 (SYNC_INCOMPLETE) and 6 (ROBOT_PARALLEL_CONFLICT) are simplified/omitted
        # for initial implementation. They can be added if needed.
        
        return {
            "success": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "topo_order": topo_order
        }


# ============================================================================
# Module Test
# ============================================================================

if __name__ == "__main__":
    print("[DB_Interactor] Module loaded successfully")
    print(f"[DB_Interactor] Database imports available: {DB_IMPORTS_AVAILABLE}")
    
    if DB_IMPORTS_AVAILABLE:
        # Test instantiation
        try:
            db = DB_Interactor()
            print("[DB_Interactor] Instantiation successful")
            db.close()
            print("[DB_Interactor] Close successful")
        except Exception as e:
            print(f"[DB_Interactor] Error during test: {e}")

```

## Composition Helper
```py
"""
CompositionHelper - Helper DTOs & Composition Interface (Nodes + Edges)

This module provides Data Transfer Objects (DTOs) and the CompositionHelper class
to compose Actions → Activities and Activities → Tasks on top of the node + edge model.

Clean-cut only (no legacy support). Designed to be UI and ROS-friendly.

Architecture Version: 2.0 (October 2025)
See: database/docs/refactor_1/Architecture_Change_V02.md
"""

from dataclasses import dataclass, field, asdict
from typing import Optional, List, Set, Tuple, Dict, Any
import json

from DatabaseHandler import DatabaseHandler
from DatabaseObjects import (
    DBObj_Action,
    DBObj_Activity,
    DBObj_ActivityActionRel,
    DBObj_ActivityActionEdge,
    DBObj_Task,
    DBObj_TaskActivityRel,
    DBObj_Robot,
    DBObj_ActionDataPoint,
    DBObj_ActionDataPointCoords,
    DBObj_ActionType
)


# ============================================================================
# Exception Classes
# ============================================================================

class CompositionError(Exception):
    """Base exception for composition-related errors."""
    pass


class ValidationError(CompositionError):
    """Exception raised when validation fails.
    
    Attributes:
        report: ValidationReport containing all validation issues
    """
    def __init__(self, report: 'ValidationReport'):
        self.report = report
        error_messages = [f"{e.code}: {e.message}" for e in report.errors]
        super().__init__(f"Validation failed with {len(report.errors)} error(s): {'; '.join(error_messages)}")


class UnknownActionError(CompositionError):
    """Raised when an action_id cannot be resolved."""
    pass


class UnknownActivityError(CompositionError):
    """Raised when an activity_id cannot be resolved."""
    pass


class UnknownTaskError(CompositionError):
    """Raised when a task_id cannot be resolved."""
    pass


# ============================================================================
# Data Transfer Objects (DTOs)
# ============================================================================

@dataclass
class ActionRef:
    """Reference to an Action; both id and rowid may be present after resolution.
    
    Attributes:
        action_id: Human-readable action identifier
        action_rowid: Database rowid (resolved by helper; None until looked up)
    """
    action_id: str
    action_rowid: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary (excludes None rowid for cleaner JSON)."""
        result = {"action_id": self.action_id}
        if self.action_rowid is not None:
            result["action_rowid"] = self.action_rowid
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionRef':
        """Create from dictionary."""
        return cls(
            action_id=data.get("action_id", data.get("action_id", "")),
            action_rowid=data.get("action_rowid")
        )
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ActionRef':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


@dataclass
class ActionNode:
    """Activity node: an action placed inside an activity graph.
    
    Attributes:
        action_no: Unique number within activity
        ref: Reference to the action to execute
        sync_flag: None or non-negative integer group id for synchronization
    """
    action_no: int
    ref: ActionRef
    sync_flag: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {
            "action_no": self.action_no,
            "action_id": self.ref.action_id
        }
        if self.sync_flag is not None:
            result["sync_flag"] = self.sync_flag
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionNode':
        """Create from dictionary."""
        action_id = data.get("action_id", data.get("action_id", ""))
        return cls(
            action_no=data["action_no"],
            ref=ActionRef(action_id=action_id),
            sync_flag=data.get("sync_flag")
        )
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ActionNode':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


@dataclass
class Edge:
    """Directed edge between two nodes (by action_no).
    
    Attributes:
        src: Source action_no
        dst: Destination action_no
    """
    src: int
    dst: int
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {"src": self.src, "dst": self.dst}
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Edge':
        """Create from dictionary."""
        return cls(src=data["src"], dst=data["dst"])
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_json(cls, json_str: str) -> 'Edge':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


@dataclass
class ActivityPlan:
    """Complete activity plan with nodes and edges.
    
    Attributes:
        activity_id: Human-readable activity identifier
        nodes: List of action nodes in the activity
        edges: Set of directed edges (src, dst) tuples
        activity_rowid: Database rowid (resolved by helper)
    """
    activity_id: str
    nodes: List[ActionNode] = field(default_factory=list)
    edges: Set[Tuple[int, int]] = field(default_factory=set)
    activity_rowid: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "activity_id": self.activity_id,
            "nodes": [n.to_dict() for n in self.nodes],
            "edges": [{"src": src, "dst": dst} for src, dst in sorted(self.edges)]
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActivityPlan':
        """Create from dictionary."""
        nodes = [ActionNode.from_dict(n) for n in data.get("nodes", [])]
        edges_list = data.get("edges", [])
        edges = set((e["src"], e["dst"]) for e in edges_list)
        return cls(
            activity_id=data["activity_id"],
            nodes=nodes,
            edges=edges
        )
    
    def to_json(self, indent: Optional[int] = 2) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ActivityPlan':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


@dataclass
class TaskPlan:
    """Task plan with ordered activity references.
    
    Attributes:
        task_id: Human-readable task identifier
        activity_ids_in_order: Ordered list of activity IDs
        task_rowid: Database rowid (resolved by helper)
    """
    task_id: str
    activity_ids_in_order: List[str] = field(default_factory=list)
    task_rowid: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "task_id": self.task_id,
            "activity_ids_in_order": self.activity_ids_in_order
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskPlan':
        """Create from dictionary."""
        return cls(
            task_id=data["task_id"],
            activity_ids_in_order=data.get("activity_ids_in_order", [])
        )
    
    def to_json(self, indent: Optional[int] = 2) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'TaskPlan':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


@dataclass
class ValidationIssue:
    """Validation issue (error or warning).
    
    Attributes:
        code: Error code (e.g., 'DUP_ACTION_NO', 'CYCLE')
        message: Human-readable message
        details: Additional context dictionary
    """
    code: str
    message: str
    details: Dict[str, object] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {"code": self.code, "message": self.message}
        if self.details:
            result["details"] = self.details
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ValidationIssue':
        """Create from dictionary."""
        return cls(
            code=data["code"],
            message=data["message"],
            details=data.get("details", {})
        )


@dataclass
class ValidationReport:
    """Validation result report.
    
    Attributes:
        success: True if validation passed (no errors)
        errors: List of blocking validation issues
        warnings: List of non-blocking validation issues
        topo_order: Computed topological order of action_nos (if DAG is valid)
    """
    success: bool
    errors: List[ValidationIssue] = field(default_factory=list)
    warnings: List[ValidationIssue] = field(default_factory=list)
    topo_order: List[int] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "success": self.success,
            "errors": [e.to_dict() for e in self.errors],
            "warnings": [w.to_dict() for w in self.warnings],
            "topo_order": self.topo_order
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ValidationReport':
        """Create from dictionary."""
        return cls(
            success=data["success"],
            errors=[ValidationIssue.from_dict(e) for e in data.get("errors", [])],
            warnings=[ValidationIssue.from_dict(w) for w in data.get("warnings", [])],
            topo_order=data.get("topo_order", [])
        )
    
    def to_json(self, indent: Optional[int] = 2) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ValidationReport':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


# ============================================================================
# Database Manager DTOs (for CRUD operations)
# ============================================================================

@dataclass
class ActionDetail:
    """Detailed action information for Database Manager UI.
    
    Attributes:
        rowid: Database rowid
        action_id: Human-readable action identifier
        robot_id: Robot identifier (None if no robot assigned)
        action_type: Action type identifier (e.g., 'Trajectory', 'MoveToConfig')
        duration_ms: Duration in milliseconds (None if not calculated)
        waypoint_count: Number of waypoints (0 if no data points)
        created_time_ms: Creation timestamp in milliseconds (None if not set)
    """
    rowid: int
    action_id: str
    robot_id: Optional[str] = None
    action_type: Optional[str] = None
    duration_ms: Optional[int] = None
    waypoint_count: int = 0
    created_time_ms: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "rowid": self.rowid,
            "action_id": self.action_id,
            "robot_id": self.robot_id,
            "action_type": self.action_type,
            "duration_ms": self.duration_ms,
            "waypoint_count": self.waypoint_count,
            "created_time_ms": self.created_time_ms
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionDetail':
        """Create from dictionary."""
        return cls(
            rowid=data["rowid"],
            action_id=data["action_id"],
            robot_id=data.get("robot_id"),
            action_type=data.get("action_type"),
            duration_ms=data.get("duration_ms"),
            waypoint_count=data.get("waypoint_count", 0),
            created_time_ms=data.get("created_time_ms")
        )


@dataclass
class ActivityGraph:
    """Activity graph structure (nodes + edges).
    
    Attributes:
        nodes: List of node dictionaries with action_no, action_id, sync_flag
        edges: List of edge dictionaries with source, destination
    """
    nodes: List[Dict[str, Any]] = field(default_factory=list)
    edges: List[Dict[str, Any]] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "nodes": self.nodes,
            "edges": self.edges
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActivityGraph':
        """Create from dictionary."""
        return cls(
            nodes=data.get("nodes", []),
            edges=data.get("edges", [])
        )
    
    @classmethod
    def from_activity_plan(cls, plan: ActivityPlan) -> 'ActivityGraph':
        """Create from ActivityPlan DTO.
        
        Args:
            plan: ActivityPlan to convert
        
        Returns:
            ActivityGraph with nodes and edges in UI format
        """
        nodes = [
            {
                "action_no": node.action_no,
                "action_id": node.ref.action_id,
                "sync_flag": node.sync_flag
            }
            for node in plan.nodes
        ]
        
        edges = [
            {
                "source": src,
                "destination": dst
            }
            for src, dst in sorted(plan.edges)
        ]
        
        return cls(nodes=nodes, edges=edges)


@dataclass
class ActivityDetail:
    """Detailed activity information for Database Manager UI.
    
    Attributes:
        rowid: Database rowid
        activity_id: Human-readable activity identifier
        graph: Activity graph with nodes and edges
        node_count: Number of nodes in the graph
        edge_count: Number of edges in the graph
        activitytime: Total activity time in milliseconds (0 if not calculated)
    """
    rowid: int
    activity_id: str
    graph: ActivityGraph = field(default_factory=ActivityGraph)
    node_count: int = 0
    edge_count: int = 0
    activitytime: int = 0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "rowid": self.rowid,
            "activity_id": self.activity_id,
            "graph": self.graph.to_dict(),
            "node_count": self.node_count,
            "edge_count": self.edge_count,
            "activitytime": self.activitytime
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActivityDetail':
        """Create from dictionary."""
        graph_data = data.get("graph", {})
        return cls(
            rowid=data["rowid"],
            activity_id=data["activity_id"],
            graph=ActivityGraph.from_dict(graph_data) if graph_data else ActivityGraph(),
            node_count=data.get("node_count", 0),
            edge_count=data.get("edge_count", 0),
            activitytime=data.get("activitytime", 0)
        )


@dataclass
class TaskDetail:
    """Detailed task information for Database Manager UI.
    
    Attributes:
        rowid: Database rowid
        task_id: Human-readable task identifier
        activity_ids: Ordered list of activity identifiers
        tasktime: Total task time in milliseconds (0 if not calculated)
    """
    rowid: int
    task_id: str
    activity_ids: List[str] = field(default_factory=list)
    tasktime: int = 0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "rowid": self.rowid,
            "task_id": self.task_id,
            "activity_ids": self.activity_ids,
            "tasktime": self.tasktime
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskDetail':
        """Create from dictionary."""
        return cls(
            rowid=data["rowid"],
            task_id=data["task_id"],
            activity_ids=data.get("activity_ids", []),
            tasktime=data.get("tasktime", 0)
        )


@dataclass
class DependencyInfo:
    """Information about a dependency relationship.
    
    Attributes:
        type: Dependency type ('activity' or 'task')
        id: Identifier of the dependent entity
        name: Human-readable name for UI display
    """
    type: str  # 'activity' or 'task'
    id: str
    name: str
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "type": self.type,
            "id": self.id,
            "name": self.name
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DependencyInfo':
        """Create from dictionary."""
        return cls(
            type=data["type"],
            id=data["id"],
            name=data["name"]
        )


@dataclass
class DeleteResult:
    """Result of a delete operation with optional dependency warnings.
    
    Attributes:
        success: True if deletion succeeded, False if blocked by dependencies
        dependencies: List of dependent entities (empty if success=True)
        warning: Human-readable warning message (None if success=True)
    """
    success: bool
    dependencies: List[DependencyInfo] = field(default_factory=list)
    warning: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            "success": self.success,
            "dependencies": [d.to_dict() for d in self.dependencies]
        }
        if self.warning:
            result["warning"] = self.warning
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DeleteResult':
        """Create from dictionary."""
        return cls(
            success=data["success"],
            dependencies=[DependencyInfo.from_dict(d) for d in data.get("dependencies", [])],
            warning=data.get("warning")
        )


# ============================================================================
# CompositionHelper Class
# ============================================================================

class CompositionHelper:
    """High-level helper to compose Actions→Activities and Activities→Tasks.
    
    Uses the node+edge model with replace-on-save semantics. Foreign keys must be enabled.
    Clean-cut only - no legacy support.
    
    Architecture Version: 2.0 (October 2025)
    """
    
    def __init__(self, dbh: DatabaseHandler):
        """Initialize with a DatabaseHandler.
        
        Args:
            dbh: Database connection manager (no ownership taken)
        
        Raises:
            ValueError: If dbh is None or not a DatabaseHandler
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            raise ValueError("dbh must be a valid DatabaseHandler instance")
        self.dbh = dbh
    
    # ========================================================================
    # Catalog / Lookup
    # ========================================================================
    
    def list_actions(
        self,
        *,
        robot_id: Optional[str] = None,
        actiontype_id: Optional[str] = None,
        search: Optional[str] = None
    ) -> List[ActionRef]:
        """Return a filtered list of available actions for UI pickers.
        
        Args:
            robot_id: Filter by robot (if specified)
            actiontype_id: Filter by action type (if specified)
            search: Filter by action_id substring (if specified)
        
        Returns:
            List of ActionRef objects with resolved rowids
        """
        # Fetch all actions from database
        all_actions = DBObj_Action.fetch_all(self.dbh)
        
        result = []
        for action in all_actions:
            action_id = action[1]  # action_id is second column
            action_rowid = action[0]  # rowid is first column
            
            # Apply filters
            if search and search.lower() not in action_id.lower():
                continue
            
            # TODO: Add robot_id and actiontype_id filtering when needed
            # This requires JOIN queries or additional filtering logic
            
            result.append(ActionRef(action_id=action_id, action_rowid=action_rowid))
        
        return result
    
    def resolve_action_ids(self, action_ids: List[str]) -> Dict[str, int]:
        """Resolve action_ids to rowids.
        
        Args:
            action_ids: List of action identifiers to resolve
        
        Returns:
            Dictionary mapping action_id → rowid
        
        Raises:
            UnknownActionError: If any action_id is unknown
        """
        result = {}
        unknown = []
        
        for action_id in action_ids:
            row = DBObj_Action.fetch_one(self.dbh, action_id)
            if row is None:
                unknown.append(action_id)
            else:
                result[action_id] = row[0]  # rowid is first column
        
        if unknown:
            raise UnknownActionError(f"Unknown action_id(s): {', '.join(unknown)}")
        
        return result
    
    # ========================================================================
    # Activity Load/Save
    # ========================================================================
    
    def get_activity_plan(self, activity_id: str) -> ActivityPlan:
        """Load nodes + edges of an activity, resolving action_ids and rowids.
        
        Args:
            activity_id: Activity identifier
        
        Returns:
            ActivityPlan with resolved nodes and edges
        
        Raises:
            UnknownActivityError: If activity_id doesn't exist
        """
        # Resolve activity
        activity_row = DBObj_Activity.fetch_one(self.dbh, activity_id)
        if activity_row is None:
            raise UnknownActivityError(f"Activity '{activity_id}' not found")
        
        activity_rowid = activity_row[0]
        
        # Fetch nodes (returns list of tuples: (action_no, action_id, sync_flag))
        nodes_data = DBObj_ActivityActionRel.fetch_nodes(self.dbh, activity_id)
        nodes = []
        for action_no, action_id, sync_flag in nodes_data:
            # Resolve action rowid
            action_row = DBObj_Action.fetch_one(self.dbh, action_id)
            action_rowid = action_row[0] if action_row else None
            
            nodes.append(ActionNode(
                action_no=action_no,
                ref=ActionRef(action_id=action_id, action_rowid=action_rowid),
                sync_flag=sync_flag
            ))
        
        # Fetch edges (returns list of tuples: (activity_id, src_action_no, dst_action_no))
        edges_data = DBObj_ActivityActionEdge.fetch_by_activity(self.dbh, activity_id)
        edges = set((src, dst) for _, src, dst in edges_data)
        
        return ActivityPlan(
            activity_id=activity_id,
            nodes=nodes,
            edges=edges,
            activity_rowid=activity_rowid
        )
    
    def save_activity(self, plan: ActivityPlan, *, validate: bool = True) -> None:
        """Replace-write nodes and edges of an activity inside a single transaction.
        
        Args:
            plan: ActivityPlan to save
            validate: If True, validate before saving and abort on errors
        
        Raises:
            ValidationError: If validation fails (when validate=True)
            CompositionError: If database operation fails
        """
        # Validate if requested
        if validate:
            report = self.validate_activity(plan)
            if not report.success:
                raise ValidationError(report)
        
        # Begin transaction (using context manager's autocommit=False for manual control)
        original_autocommit = self.dbh.autoCommit
        self.dbh.autoCommit = False
        
        try:
            # Resolve or create activity
            activity_row = DBObj_Activity.fetch_one(self.dbh, plan.activity_id)
            if activity_row is None:
                # Create new activity with default time
                DBObj_Activity.insert_one(self.dbh, plan.activity_id, activitytime=0)
                activity_row = DBObj_Activity.fetch_one(self.dbh, plan.activity_id)
            
            activity_rowid = activity_row[0]
            plan.activity_rowid = activity_rowid
            
            # Delete existing edges and nodes (in correct order due to FK constraints)
            DBObj_ActivityActionEdge.delete_by_activity(self.dbh, plan.activity_id)
            
            # Delete nodes by activity_rowid
            sqlquery = f"DELETE FROM {DBObj_ActivityActionRel.tablename} WHERE activity_rowid = {activity_rowid}"
            self.dbh.execute(sqlquery)
            
            # Insert nodes
            if plan.nodes:
                nodes_data = [
                    {
                        "action_id": node.ref.action_id,
                        "action_no": node.action_no,
                        "sync_flag": node.sync_flag
                    }
                    for node in plan.nodes
                ]
                DBObj_ActivityActionRel.insert_nodes(self.dbh, plan.activity_id, nodes_data)
            
            # Insert edges (deduplicated via set)
            if plan.edges:
                edges_list = list(plan.edges)
                DBObj_ActivityActionEdge.insert_edges(self.dbh, plan.activity_id, edges_list)
            
            # update acttivity time TODO: calculate shortest path time
            total_time = sum(
                DBObj_Action.fetch_one(self.dbh, node.ref.action_id)["duration_ms"] or 0
                for node in plan.nodes
            )
            DBObj_Activity.update_time(self.dbh, plan.activity_id, total_time)

            # Commit transaction
            self.dbh.commit()
            
        except Exception as e:
            # Rollback on error
            self.dbh.rollback()
            raise CompositionError(f"Failed to save activity: {e}") from e
        
        finally:
            # Restore original autocommit setting
            self.dbh.autoCommit = original_autocommit
    
    # ========================================================================
    # Activity Building / Editing
    # ========================================================================
    
    def new_activity_plan(self, activity_id: str) -> ActivityPlan:
        """Create a new empty activity plan.
        
        Args:
            activity_id: Activity identifier
        
        Returns:
            Empty ActivityPlan
        """
        return ActivityPlan(activity_id=activity_id)
    
    def add_node(
        self,
        plan: ActivityPlan,
        action_id: str,
        action_no: int,
        *,
        sync_flag: Optional[int] = None
    ) -> None:
        """Add a node to the activity plan.
        
        Args:
            plan: ActivityPlan to modify
            action_id: Action to add
            action_no: Unique node number
            sync_flag: Optional synchronization group
        
        Raises:
            ValueError: If action_no already exists
        """
        # Check for duplicate action_no
        if any(n.action_no == action_no for n in plan.nodes):
            raise ValueError(f"action_no {action_no} already exists in plan")
        
        node = ActionNode(
            action_no=action_no,
            ref=ActionRef(action_id=action_id),
            sync_flag=sync_flag
        )
        plan.nodes.append(node)
    
    def remove_node(self, plan: ActivityPlan, action_no: int) -> None:
        """Remove a node and all its connected edges from the plan.
        
        Args:
            plan: ActivityPlan to modify
            action_no: Node number to remove
        
        Raises:
            ValueError: If action_no doesn't exist
        """
        # Find and remove node
        original_len = len(plan.nodes)
        plan.nodes = [n for n in plan.nodes if n.action_no != action_no]
        
        if len(plan.nodes) == original_len:
            raise ValueError(f"action_no {action_no} not found in plan")
        
        # Remove all edges connected to this node
        plan.edges = {(src, dst) for src, dst in plan.edges if src != action_no and dst != action_no}
    
    def connect(self, plan: ActivityPlan, src: int, dst: int) -> None:
        """Add edge (src→dst). No self-loops; duplicates ignored.
        
        Args:
            plan: ActivityPlan to modify
            src: Source action_no
            dst: Destination action_no
        
        Raises:
            ValueError: If attempting to create self-loop
        """
        if src == dst:
            raise ValueError(f"Self-loop not allowed: {src}→{dst}")
        
        plan.edges.add((src, dst))
    
    def disconnect(self, plan: ActivityPlan, src: int, dst: int) -> None:
        """Remove edge (src→dst) if it exists.
        
        Args:
            plan: ActivityPlan to modify
            src: Source action_no
            dst: Destination action_no
        """
        plan.edges.discard((src, dst))
    
    def set_syncflag(
        self,
        plan: ActivityPlan,
        action_nos: List[int],
        flag: Optional[int]
    ) -> None:
        """Assign or clear a sync_flag for multiple nodes at once.
        
        Args:
            plan: ActivityPlan to modify
            action_nos: List of node numbers to update
            flag: Synchronization flag value (None to clear)
        
        Raises:
            ValueError: If any action_no doesn't exist
        """
        action_no_set = set(action_nos)
        found = set()
        
        for node in plan.nodes:
            if node.action_no in action_no_set:
                node.sync_flag = flag
                found.add(node.action_no)
        
        missing = action_no_set - found
        if missing:
            raise ValueError(f"action_no(s) not found: {', '.join(map(str, missing))}")
    
    def renumber(self, plan: ActivityPlan, mapping: Dict[int, int]) -> None:
        """Renumber nodes; updates edges consistently; fails on collisions.
        
        Args:
            plan: ActivityPlan to modify
            mapping: Dictionary mapping old_action_no → new_action_no
        
        Raises:
            ValueError: If renumbering would cause collisions
        """
        # Check for collisions
        old_numbers = {n.action_no for n in plan.nodes}
        new_numbers = set(mapping.values())
        unchanged_numbers = old_numbers - set(mapping.keys())
        
        collisions = new_numbers & unchanged_numbers
        if collisions:
            raise ValueError(f"Renumbering collision: new numbers {collisions} already exist")
        
        # Check for duplicate targets
        if len(new_numbers) != len(mapping):
            raise ValueError("Renumbering mapping contains duplicate target values")
        
        # Renumber nodes
        for node in plan.nodes:
            if node.action_no in mapping:
                node.action_no = mapping[node.action_no]
        
        # Renumber edges
        new_edges = set()
        for src, dst in plan.edges:
            new_src = mapping.get(src, src)
            new_dst = mapping.get(dst, dst)
            new_edges.add((new_src, new_dst))
        plan.edges = new_edges
    
    # ========================================================================
    # Validation
    # ========================================================================
    
    def validate_activity(self, plan: ActivityPlan) -> ValidationReport:
        """Check uniqueness, existence, DAG property, sync_flag path rule, and robot conflicts.
        
        Args:
            plan: ActivityPlan to validate
        
        Returns:
            ValidationReport with errors, warnings, and topological order
        
        Robot Conflict Rule:
            Warns if the same robot is used in parallel branches (nodes with no path between them).
            This prevents physical robot conflicts in concurrent execution.
        """
        errors: List[ValidationIssue] = []
        warnings: List[ValidationIssue] = []
        topo_order: List[int] = []
        
        # 1) Uniqueness of action_no
        seen = set()
        for node in plan.nodes:
            if node.action_no in seen:
                errors.append(ValidationIssue(
                    'DUP_ACTION_NO',
                    f'Duplicate action_no: {node.action_no}'
                ))
            seen.add(node.action_no)
        
        # 2) Known actions (resolve rowids)
        action_ids = [n.ref.action_id for n in plan.nodes]
        try:
            id2rowid = self.resolve_action_ids(action_ids)
            # Update node refs with resolved rowids
            for node in plan.nodes:
                node.ref.action_rowid = id2rowid.get(node.ref.action_id)
        except UnknownActionError as e:
            # Extract unknown IDs from error message
            for node in plan.nodes:
                row = DBObj_Action.fetch_one(self.dbh, node.ref.action_id)
                if row is None:
                    errors.append(ValidationIssue(
                        'UNKNOWN_ACTION',
                        f'Unknown action_id: {node.ref.action_id}'
                    ))
        
        # 3) Edge endpoints & basic checks
        nodeset = {n.action_no for n in plan.nodes}
        edges_validated = set()
        
        for src, dst in plan.edges:
            # Self-loop check
            if src == dst:
                errors.append(ValidationIssue(
                    'SELF_LOOP',
                    f'Self-loop at action_no {src}'
                ))
                continue
            
            # Endpoint existence check
            if src not in nodeset:
                errors.append(ValidationIssue(
                    'EDGE_ENDPOINT_MISSING',
                    f'Edge ({src}→{dst}) references missing source node {src}'
                ))
                continue
            
            if dst not in nodeset:
                errors.append(ValidationIssue(
                    'EDGE_ENDPOINT_MISSING',
                    f'Edge ({src}→{dst}) references missing destination node {dst}'
                ))
                continue
            
            # Duplicate check (should not happen with set, but for safety)
            if (src, dst) in edges_validated:
                errors.append(ValidationIssue(
                    'DUP_EDGE',
                    f'Duplicate edge ({src}→{dst})'
                ))
                continue
            
            edges_validated.add((src, dst))
        
        # 4) DAG check (Kahn's algorithm for topological sort)
        if not errors:  # Only if no errors so far
            indeg = {i: 0 for i in nodeset}
            edges_copy = set(edges_validated)
            
            # Calculate in-degrees
            for src, dst in edges_copy:
                indeg[dst] += 1
            
            # Initialize queue with nodes of in-degree 0
            queue = [i for i in nodeset if indeg[i] == 0]
            topo_order = []
            
            while queue:
                # Sort for deterministic ordering
                queue.sort()
                current = queue.pop(0)
                topo_order.append(current)
                
                # Remove edges from current node
                edges_to_remove = [(src, dst) for src, dst in edges_copy if src == current]
                for src, dst in edges_to_remove:
                    edges_copy.remove((src, dst))
                    indeg[dst] -= 1
                    if indeg[dst] == 0:
                        queue.append(dst)
            
            # Check for cycles
            if edges_copy:
                errors.append(ValidationIssue(
                    'CYCLE',
                    f'Cycle detected in activity graph (remaining edges: {len(edges_copy)})',
                    details={'remaining_edges': list(edges_copy)}
                ))
                topo_order = []  # Clear topo order if cycle detected
        
        # 5) sync_flag path rule
        if not errors:  # Only if no errors so far
            # Group nodes by sync_flag
            by_flag: Dict[int, Set[int]] = {}
            for node in plan.nodes:
                if node.sync_flag is not None:
                    by_flag.setdefault(node.sync_flag, set()).add(node.action_no)
            
            # Build adjacency list for reachability checks
            adj: Dict[int, Set[int]] = {i: set() for i in nodeset}
            for src, dst in edges_validated:
                adj[src].add(dst)
            
            def reachable_from(start: int) -> Set[int]:
                """Find all nodes reachable from start node."""
                seen = set()
                stack = [start]
                while stack:
                    current = stack.pop()
                    for neighbor in adj[current]:
                        if neighbor not in seen:
                            seen.add(neighbor)
                            stack.append(neighbor)
                return seen
            
            # Check each sync_flag group
            for flag, group in by_flag.items():
                checked_pairs = set()
                for u in group:
                    reach = reachable_from(u)
                    for v in group:
                        if v != u and v in reach:
                            pair = tuple(sorted([u, v]))
                            if pair not in checked_pairs:
                                checked_pairs.add(pair)
                                errors.append(ValidationIssue(
                                    'SYNCFLAG_PATH',
                                    f'Path exists within sync_flag={flag} between nodes {u} and {v}',
                                    details={'u': u, 'v': v, 'flag': flag}
                                ))
        
        # 6) Robot conflict check (same robot in parallel branches)
        if not errors:  # Only if no errors so far
            # Map action_no → robot_id
            node_to_robot: Dict[int, Optional[str]] = {}
            for node in plan.nodes:
                # Fetch action details to get robot_rowid
                action_row = DBObj_Action.fetch_one(self.dbh, node.ref.action_id)
                if action_row:
                    robot_rowid = action_row["robot_rowid"]
                    # Resolve robot_rowid → robot_id
                    robot_row = DBObj_Robot.fetch_one_by_row_id(self.dbh, robot_rowid)
                    if robot_row:
                        robot_id = robot_row["robot_id"]
                        node_to_robot[node.action_no] = robot_id
                    else:
                        node_to_robot[node.action_no] = None
                else:
                    node_to_robot[node.action_no] = None
            
            # Build reverse adjacency list for reachability checks
            rev_adj: Dict[int, Set[int]] = {i: set() for i in nodeset}
            for src, dst in edges_validated:
                rev_adj[dst].add(src)
            
            def has_path_between(u: int, v: int) -> bool:
                """Check if there's any path between u and v (in either direction)."""
                # Check u → v
                if v in reachable_from(u):
                    return True
                # Check v → u (using reverse adjacency)
                seen = set()
                stack = [u]
                while stack:
                    current = stack.pop()
                    for parent in rev_adj[current]:
                        if parent not in seen:
                            if parent == v:
                                return True
                            seen.add(parent)
                            stack.append(parent)
                return False
            
            # Group nodes by robot_id
            by_robot: Dict[str, List[int]] = {}
            for action_no, robot_id in node_to_robot.items():
                if robot_id:  # Skip nodes without valid robot
                    by_robot.setdefault(robot_id, []).append(action_no)
            
            # Check for parallel usage of same robot
            for robot_id, nodes_list in by_robot.items():
                if len(nodes_list) < 2:
                    continue
                
                # Check all pairs of nodes with same robot
                checked_pairs = set()
                for i, u in enumerate(nodes_list):
                    for v in nodes_list[i+1:]:
                        pair = tuple(sorted([u, v]))
                        if pair not in checked_pairs:
                            checked_pairs.add(pair)
                            # If no path exists between them, they're parallel
                            if not has_path_between(u, v):
                                warnings.append(ValidationIssue(
                                    'ROBOT_PARALLEL_CONFLICT',
                                    f'Robot "{robot_id}" used in parallel branches (nodes {u} and {v} have no path between them)',
                                    details={'robot_id': robot_id, 'node_u': u, 'node_v': v}
                                ))
        
        # Construct report
        return ValidationReport(
            success=(len(errors) == 0),
            errors=errors,
            warnings=warnings,
            topo_order=topo_order
        )
    
    # ========================================================================
    # Task Load/Save
    # ========================================================================
    
    def get_task_plan(self, task_id: str) -> TaskPlan:
        """Load task plan with ordered activity references.
        
        Args:
            task_id: Task identifier
        
        Returns:
            TaskPlan with ordered activity IDs
        
        Raises:
            UnknownTaskError: If task_id doesn't exist
        """
        # Resolve task
        task_row = DBObj_Task.fetch_one(self.dbh, task_id)
        if task_row is None:
            raise UnknownTaskError(f"Task '{task_id}' not found")
        
        task_rowid = task_row[0]
        
        # Fetch task-activity relationships (ordered by rowid = insertion order)
        # Returns list of tuples: (rowid, task_rowid, activity_rowid)
        rels = DBObj_TaskActivityRel.fetch_by_task(self.dbh, task_id)
        
        # Resolve activity_rowid → activity_id
        activity_ids = []
        for rel in rels:
            activity_rowid = rel[2]  # Third column
            # Fetch activity_id by rowid using SQL query
            sqlquery = f"SELECT activity_id FROM {DBObj_Activity.tablename} WHERE rowid = {activity_rowid}"
            result = self.dbh.execute(sqlquery).fetchone()
            if result:
                activity_id = result[0]
                activity_ids.append(activity_id)
        
        return TaskPlan(
            task_id=task_id,
            activity_ids_in_order=activity_ids,
            task_rowid=task_rowid
        )
    
    def save_task(self, task: TaskPlan) -> None:
        """Replace-write ordered activity mapping for a task.
        
        Args:
            task: TaskPlan to save
        
        Raises:
            UnknownActivityError: If any activity_id doesn't exist
            CompositionError: If database operation fails
        """
        original_autocommit = self.dbh.autoCommit
        self.dbh.autoCommit = False
        
        try:
            # Resolve or create task
            task_row = DBObj_Task.fetch_one(self.dbh, task.task_id)
            if task_row is None:
                # Create new task with default time
                DBObj_Task.insert_one(self.dbh, task.task_id, tasktime=0)
                task_row = DBObj_Task.fetch_one(self.dbh, task.task_id)
            
            task_rowid = task_row[0]
            task.task_rowid = task_rowid
            
            # Delete existing task-activity relationships
            sqlquery = f"DELETE FROM {DBObj_TaskActivityRel.tablename} WHERE task_rowid = {task_rowid}"
            self.dbh.execute(sqlquery)
            
            # Insert activities in order (rowid = insertion order determines sequence)
            if task.activity_ids_in_order:
                list_of_dict = [{"activity_id": aid} for aid in task.activity_ids_in_order]
                DBObj_TaskActivityRel.insert_by_task(self.dbh, task.task_id, list_of_dict)
            
            # Commit transaction
            self.dbh.commit()
            
        except Exception as e:
            # Rollback on error
            self.dbh.rollback()
            raise CompositionError(f"Failed to save task: {e}") from e
        
        finally:
            # Restore original autocommit setting
            self.dbh.autoCommit = original_autocommit
    
    # ========================================================================
    # Database Manager - High-Level CRUD Operations
    # ========================================================================
    
    def list_actions_full(self, search: Optional[str] = None) -> List[ActionDetail]:
        """List all actions with full details for Database Manager UI.
        
        Args:
            search: Optional substring filter for action_id
        
        Returns:
            List of ActionDetail objects with all metadata
        """
        # Fetch all actions
        all_actions = DBObj_Action.fetch_all(self.dbh)
        
        result = []
        for action_row in all_actions:
            rowid = action_row["rowid"]
            action_id = action_row["action_id"]
            
            # Apply search filter
            if search and search.lower() not in action_id.lower():
                continue
            
            # Resolve robot_id from robot_rowid
            robot_id = None
            robot_rowid = action_row["robot_rowid"]
            if robot_rowid:
                robot_row = DBObj_Robot.fetch_one_by_row_id(self.dbh, robot_rowid)
                if robot_row:
                    robot_id = robot_row["robot_id"]
            
            # Get action_type from actiontype_rowid
            action_type = None
            actiontype_rowid = action_row["actiontype_rowid"]
            if actiontype_rowid:
                actiontype_row = DBObj_ActionType.fetch_one_by_row_id(self.dbh, actiontype_rowid)
                if actiontype_row:
                    action_type = actiontype_row["actiontype_id"]
            
            # Get metadata from Action table (sqlite3.Row doesn't support .get())
            try:
                duration_ms = action_row["duration_ms"]
            except (KeyError, IndexError):
                duration_ms = None
                
            try:
                waypoint_count = action_row["waypoint_count"]
            except (KeyError, IndexError):
                waypoint_count = 0
            
            # Get created timestamp from metadata JSON if available
            created_time_ms = None
            try:
                metadata_json = action_row["metadata"]
                if metadata_json:
                    metadata = json.loads(metadata_json)
                    created_time_ms = metadata.get("creation_time_ms")
            except (KeyError, IndexError, json.JSONDecodeError):
                pass
            
            result.append(ActionDetail(
                rowid=rowid,
                action_id=action_id,
                robot_id=robot_id,
                action_type=action_type,
                duration_ms=duration_ms,
                waypoint_count=waypoint_count,
                created_time_ms=created_time_ms
            ))
        
        return result
    
    def list_activities_full(self) -> List[ActivityDetail]:
        """List all activities with full details including graph for Database Manager UI.
        
        Returns:
            List of ActivityDetail objects with graph, node/edge counts
        """
        # Fetch all activities
        all_activities = DBObj_Activity.fetch_all(self.dbh)
        
        result = []
        for activity_row in all_activities:
            rowid = activity_row["rowid"]
            activity_id = activity_row["activity_id"]
            
            # Get activitytime (sqlite3.Row doesn't support .get())
            try:
                activitytime = activity_row["activitytime"]
            except (KeyError, IndexError):
                activitytime = 0
            
            # Build graph using existing get_activity_plan method
            try:
                plan = self.get_activity_plan(activity_id)
                graph = ActivityGraph.from_activity_plan(plan)
                node_count = len(plan.nodes)
                edge_count = len(plan.edges)
            except UnknownActivityError:
                # Activity exists but has no graph data
                graph = ActivityGraph()
                node_count = 0
                edge_count = 0
            
            result.append(ActivityDetail(
                rowid=rowid,
                activity_id=activity_id,
                graph=graph,
                node_count=node_count,
                edge_count=edge_count,
                activitytime=activitytime
            ))
        
        return result
    
    def list_tasks_full(self) -> List[TaskDetail]:
        """List all tasks with full details for Database Manager UI.
        
        Returns:
            List of TaskDetail objects with activity_ids and tasktime
        """
        # Fetch all tasks
        all_tasks = DBObj_Task.fetch_all(self.dbh)
        
        result = []
        for task_row in all_tasks:
            rowid = task_row["rowid"]
            task_id = task_row["task_id"]
            
            # Get tasktime (sqlite3.Row doesn't support .get())
            try:
                tasktime = task_row["tasktime"]
            except (KeyError, IndexError):
                tasktime = 0
            
            # Get activity IDs using existing get_task_plan method
            try:
                plan = self.get_task_plan(task_id)
                activity_ids = plan.activity_ids_in_order
            except UnknownTaskError:
                # Task exists but has no activities
                activity_ids = []
            
            result.append(TaskDetail(
                rowid=rowid,
                task_id=task_id,
                activity_ids=activity_ids,
                tasktime=tasktime
            ))
        
        return result
    
    def update_action(
        self,
        rowid: int,
        *,
        action_id: Optional[str] = None,
        robot_id: Optional[str] = None
    ) -> ActionDetail:
        """Update action fields by rowid.
        
        Args:
            rowid: Database rowid of the action
            action_id: New action_id (None to keep unchanged)
            robot_id: New robot_id (None to keep unchanged)
        
        Returns:
            Updated ActionDetail with optional warning in metadata
        
        Raises:
            CompositionError: If action not found or update fails
            ValueError: If new action_id already exists (duplicate)
        
        Note:
            If action_id is changed and action is used in activities,
            a warning is logged but update proceeds (references use rowid).
        """
        # Check if action exists
        action_row = DBObj_Action.fetch_one_by_row_id(self.dbh, rowid)
        if not action_row:
            raise CompositionError(f"Action with rowid {rowid} not found")
        
        current_action_id = action_row["action_id"]
        
        # Check for dependencies if changing action_id
        dependencies = []
        if action_id and action_id != current_action_id:
            dependencies = self.get_action_dependencies(rowid)
            if dependencies:
                dep_names = [f"{d.type}:{d.name}" for d in dependencies]
                print(f"[WARN] Changing action_id '{current_action_id}' to '{action_id}' - "
                      f"Used in {len(dependencies)} items: {', '.join(dep_names)}")
        
        # Check for duplicate action_id if changing
        if action_id and action_id != current_action_id:
            existing = DBObj_Action.fetch_one(self.dbh, action_id)
            if existing:
                raise ValueError(f"Action ID '{action_id}' already exists")
        
        # Build UPDATE query
        update_fields = []
        params = []
        
        if action_id and action_id != current_action_id:
            update_fields.append("action_id = ?")
            params.append(action_id)
        
        if robot_id is not None:
            # Resolve robot_id to robot_rowid
            robot_row = DBObj_Robot.fetch_one(self.dbh, robot_id)
            if not robot_row:
                raise ValueError(f"Robot ID '{robot_id}' not found")
            update_fields.append("robot_rowid = ?")
            params.append(robot_row["rowid"])
        
        if update_fields:
            update_query = f"UPDATE {DBObj_Action.tablename} SET {', '.join(update_fields)} WHERE rowid = ?"
            params.append(rowid)
            self.dbh.execute(update_query, params)
            self.dbh.commit()
        
        # Return updated action details
        updated_row = DBObj_Action.fetch_one_by_row_id(self.dbh, rowid)
        
        # Build ActionDetail from updated row
        robot_id_resolved = None
        if updated_row["robot_rowid"]:
            robot_row = DBObj_Robot.fetch_one_by_row_id(self.dbh, updated_row["robot_rowid"])
            if robot_row:
                robot_id_resolved = robot_row["robot_id"]
        
        return ActionDetail(
            rowid=rowid,
            action_id=updated_row["action_id"],
            robot_id=robot_id_resolved,
            action_type=None,  # Not editable via this method
            duration_ms=updated_row["duration_ms"],
            waypoint_count=updated_row["waypoint_count"],
            created_time_ms=None
        )
    
    def update_activity(self, rowid: int, *, activity_id: str) -> ActivityDetail:
        """Update activity_id by rowid.
        
        Args:
            rowid: Database rowid of the activity
            activity_id: New activity_id
        
        Returns:
            Updated ActivityDetail
        
        Raises:
            CompositionError: If activity not found or update fails
            ValueError: If new activity_id already exists (duplicate)
        """
        # Check if activity exists
        activity_row = DBObj_Activity.fetch_one_by_row_id(self.dbh, rowid)
        if not activity_row:
            raise CompositionError(f"Activity with rowid {rowid} not found")
        
        current_activity_id = activity_row["activity_id"]
        
        # Check for duplicate activity_id if changing
        if activity_id != current_activity_id:
            existing = DBObj_Activity.fetch_one(self.dbh, activity_id)
            if existing:
                raise ValueError(f"Activity ID '{activity_id}' already exists")
            
            # Update activity_id
            update_query = f"UPDATE {DBObj_Activity.tablename} SET activity_id = ? WHERE rowid = ?"
            self.dbh.execute(update_query, [activity_id, rowid])
            self.dbh.commit()
        
        # Return updated activity details
        updated_row = DBObj_Activity.fetch_one_by_row_id(self.dbh, rowid)
        
        # Build graph
        try:
            plan = self.get_activity_plan(updated_row["activity_id"])
            graph = ActivityGraph.from_activity_plan(plan)
            node_count = len(plan.nodes)
            edge_count = len(plan.edges)
        except UnknownActivityError:
            graph = ActivityGraph()
            node_count = 0
            edge_count = 0
        
        return ActivityDetail(
            rowid=rowid,
            activity_id=updated_row["activity_id"],
            graph=graph,
            node_count=node_count,
            edge_count=edge_count,
            activitytime=updated_row["activitytime"]
        )
    
    def update_task(self, rowid: int, *, task_id: str) -> TaskDetail:
        """Update task_id by rowid.
        
        Args:
            rowid: Database rowid of the task
            task_id: New task_id
        
        Returns:
            Updated TaskDetail
        
        Raises:
            CompositionError: If task not found or update fails
            ValueError: If new task_id already exists (duplicate)
        """
        # Check if task exists
        task_row = DBObj_Task.fetch_one_by_row_id(self.dbh, rowid)
        if not task_row:
            raise CompositionError(f"Task with rowid {rowid} not found")
        
        current_task_id = task_row["task_id"]
        
        # Check for duplicate task_id if changing
        if task_id != current_task_id:
            existing = DBObj_Task.fetch_one(self.dbh, task_id)
            if existing:
                raise ValueError(f"Task ID '{task_id}' already exists")
            
            # Update task_id
            update_query = f"UPDATE {DBObj_Task.tablename} SET task_id = ? WHERE rowid = ?"
            self.dbh.execute(update_query, [task_id, rowid])
            self.dbh.commit()
        
        # Return updated task details
        updated_row = DBObj_Task.fetch_one_by_row_id(self.dbh, rowid)
        
        # Get activity IDs
        try:
            plan = self.get_task_plan(updated_row["task_id"])
            activity_ids = plan.activity_ids_in_order
        except UnknownTaskError:
            activity_ids = []
        
        return TaskDetail(
            rowid=rowid,
            task_id=updated_row["task_id"],
            activity_ids=activity_ids,
            tasktime=updated_row.get("tasktime", 0)
        )
    
    def get_action_dependencies(self, action_rowid: int) -> List[DependencyInfo]:
        """Find all activities that use this action.
        
        Args:
            action_rowid: Database rowid of the action
        
        Returns:
            List of DependencyInfo for activities using this action
        
        Raises:
            CompositionError: If action not found
        """
        # Get action_id from rowid
        action_row = DBObj_Action.fetch_one_by_row_id(self.dbh, action_rowid)
        if not action_row:
            raise CompositionError(f"Action with rowid {action_rowid} not found")
        
        action_id = action_row["action_id"]
        
        # Find activities that reference this action
        # Use SQL JOIN to find ActivityActionRel entries
        query = f"""
            SELECT DISTINCT a.activity_id
            FROM {DBObj_Activity.tablename} a
            JOIN {DBObj_ActivityActionRel.tablename} aar ON a.rowid = aar.activity_rowid
            JOIN {DBObj_Action.tablename} act ON aar.action_rowid_act = act.rowid
            WHERE act.action_id = ?
        """
        cursor = self.dbh.execute(query, [action_id])
        rows = cursor.fetchall()
        
        dependencies = []
        for row in rows:
            activity_id = row[0]
            dependencies.append(DependencyInfo(
                type='activity',
                id=activity_id,
                name=f"Activity: {activity_id}"
            ))
        
        return dependencies
    
    def get_activity_dependencies(self, activity_rowid: int) -> List[DependencyInfo]:
        """Find all tasks that use this activity.
        
        Args:
            activity_rowid: Database rowid of the activity
        
        Returns:
            List of DependencyInfo for tasks using this activity
        
        Raises:
            CompositionError: If activity not found
        """
        # Get activity_id from rowid
        activity_row = DBObj_Activity.fetch_one_by_row_id(self.dbh, activity_rowid)
        if not activity_row:
            raise CompositionError(f"Activity with rowid {activity_rowid} not found")
        
        activity_id = activity_row["activity_id"]
        
        # Find tasks that reference this activity
        # Use SQL JOIN to find TaskActivityRel entries
        query = f"""
            SELECT DISTINCT t.task_id
            FROM {DBObj_Task.tablename} t
            JOIN {DBObj_TaskActivityRel.tablename} tar ON t.rowid = tar.task_rowid
            JOIN {DBObj_Activity.tablename} a ON tar.activity_rowid = a.rowid
            WHERE a.activity_id = ?
        """
        cursor = self.dbh.execute(query, [activity_id])
        rows = cursor.fetchall()
        
        dependencies = []
        for row in rows:
            task_id = row[0]
            dependencies.append(DependencyInfo(
                type='task',
                id=task_id,
                name=f"Task: {task_id}"
            ))
        
        return dependencies
    
    def delete_action(self, rowid: int, *, force: bool = False) -> DeleteResult:
        """Delete action with dependency check.
        
        Args:
            rowid: Database rowid of the action
            force: If True, delete action AND all dependent activities (cascade)
        
        Returns:
            DeleteResult with success status and dependency information
        
        Raises:
            CompositionError: If action not found or delete fails
        """
        # Get action details
        action_row = DBObj_Action.fetch_one_by_row_id(self.dbh, rowid)
        if not action_row:
            raise CompositionError(f"Action with rowid {rowid} not found")
        
        action_id = action_row["action_id"]
        
        # Check dependencies
        dependencies = self.get_action_dependencies(rowid)
        
        if dependencies and not force:
            # Block deletion, return dependencies
            warning = f"Action '{action_id}' is used in {len(dependencies)} activit{'y' if len(dependencies) == 1 else 'ies'}"
            return DeleteResult(
                success=False,
                dependencies=dependencies,
                warning=warning
            )
        
        # If force=True and dependencies exist, cascade delete activities first
        if force and dependencies:
            for dep in dependencies:
                # Get activity rowid from dependency name (format: "Activity: activity_id")
                if dep.name.startswith("Activity: "):
                    activity_id = dep.name.replace("Activity: ", "")
                    activity_row = DBObj_Activity.fetch_one(self.dbh, activity_id)
                    if activity_row:
                        activity_rowid = activity_row["rowid"]
                        # Recursively delete activity (force=True to delete tasks too)
                        self.delete_activity(activity_rowid, force=True)
        
        # Delete action
        delete_query = f"DELETE FROM {DBObj_Action.tablename} WHERE rowid = ?"
        self.dbh.execute(delete_query, [rowid])
        self.dbh.commit()
        
        return DeleteResult(success=True)
    
    def delete_activity(self, rowid: int, *, force: bool = False) -> DeleteResult:
        """Delete activity with dependency check.
        
        Args:
            rowid: Database rowid of the activity
            force: If True, delete even if dependencies exist
        
        Returns:
            DeleteResult with success status and dependency information
        
        Raises:
            CompositionError: If activity not found or delete fails
        """
        # Get activity details
        activity_row = DBObj_Activity.fetch_one_by_row_id(self.dbh, rowid)
        if not activity_row:
            raise CompositionError(f"Activity with rowid {rowid} not found")
        
        activity_id = activity_row["activity_id"]
        
        # Check dependencies
        dependencies = self.get_activity_dependencies(rowid)
        
        if dependencies and not force:
            # Block deletion, return dependencies
            warning = f"Activity '{activity_id}' is used in {len(dependencies)} task{'s' if len(dependencies) > 1 else ''}"
            return DeleteResult(
                success=False,
                dependencies=dependencies,
                warning=warning
            )
        
        # Delete activity (cascade will handle ActivityActionRel and ActivityActionEdge)
        delete_query = f"DELETE FROM {DBObj_Activity.tablename} WHERE rowid = ?"
        self.dbh.execute(delete_query, [rowid])
        self.dbh.commit()
        
        return DeleteResult(success=True)
    
    def delete_task(self, rowid: int) -> DeleteResult:
        """Delete task (no dependency checks needed - tasks are top-level).
        
        Args:
            rowid: Database rowid of the task
        
        Returns:
            DeleteResult with success status
        
        Raises:
            CompositionError: If task not found or delete fails
        """
        # Get task details
        task_row = DBObj_Task.fetch_one_by_row_id(self.dbh, rowid)
        if not task_row:
            raise CompositionError(f"Task with rowid {rowid} not found")
        
        # Delete task (cascade will handle TaskActivityRel)
        delete_query = f"DELETE FROM {DBObj_Task.tablename} WHERE rowid = ?"
        self.dbh.execute(delete_query, [rowid])
        self.dbh.commit()
        
        return DeleteResult(success=True)
    


# ============================================================================
# Module Test (for development/debugging)
# ============================================================================

if __name__ == "__main__":
    print("CompositionHelper module loaded successfully")
    print("\n=== Data Transfer Objects (DTOs) ===")
    print("Core DTOs:")
    print("  - ActionRef, ActionNode, Edge, ActivityPlan, TaskPlan")
    print("  - ValidationIssue, ValidationReport")
    print("\nDatabase Manager DTOs:")
    print("  - ActionDetail, ActivityDetail, TaskDetail")
    print("  - ActivityGraph, DependencyInfo, DeleteResult")
    print("\n=== Main Class ===")
    print("  - CompositionHelper")
    print("\nAvailable methods:")
    print("  Activity Builder: list_actions, get_activity_plan, save_activity, validate_activity")
    print("  Database Manager: list_actions_full, list_activities_full, list_tasks_full")
    print("                    update_action, update_activity, update_task")
    print("                    delete_action, delete_activity, delete_task")
    print("                    get_action_dependencies, get_activity_dependencies")

```

## DBObj
```py
"""SQLite data-access helpers for the robotics research workspace.

The module exposes lightweight static helper classes (`DBObj_*`) that encapsulate
schema management and CRUD utilities for the workspace database. Each helper is
designed to be API-friendly so that robotics services can bootstrap and operate
without directly inlining SQL snippets.

Standard Method Patterns:
- create_tables(): Create the table schema if it doesn't exist
- drop_tables(): Remove the table and all its data
- schema_tables(): Print the table's SQL schema and column information for debugging
- fetch_one(): Retrieve a single row by its identifier
- fetch_one_by_row_id(): Retrieve a single row by its internal SQLite rowid
- fetch_all(): Retrieve all rows, optionally filtered by identifier
- insert_one(): Insert or replace a single row
- delete_one(): Delete a single row by its identifier
- print_all(): Print all rows to stdout for debugging
- default_insert(): Insert default/seed data for lookup tables
"""
try:
    from .Globals import *
    from .DatabaseHandler import DatabaseHandler
except:
    from Globals import *
    from DatabaseHandler import DatabaseHandler
import json
import math
import sqlite3
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

class DBObj_Robot:
    """Utility wrapper around the `Robot` lookup table.

    The table stores identifiers and descriptive metadata for robots that can
    participate in coordinated tasks. All methods expect a live
    :class:`DatabaseHandler` instance and keep the schema aligned across API
    consumers.
    """
    tablename : str = "Robot"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid         INTEGER PRIMARY KEY,"
                " robot_id      TEXT    NOT NULL UNIQUE,"
                " ipv4          TEXT    UNIQUE,"
                " descshort     TEXT,"
                " desclong      TEXT"
                ")"
                ).format(__class__.tablename)
        dbh.execute(sqlquery)
    
    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def fetch_one(dbh:DatabaseHandler, robot_id:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): 
            return None
        return dbh.execute("SELECT * FROM {0} WHERE robot_id = '{1}'".format(__class__.tablename, robot_id)).fetchone()

    @staticmethod
    def fetch_one_by_ipv4(dbh:DatabaseHandler, ipv4:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): 
            return None
        return dbh.execute("SELECT * FROM {0} WHERE ipv4 = '{1}'".format(__class__.tablename, ipv4)).fetchone()

    @staticmethod
    def fetch_one_by_row_id(dbh:DatabaseHandler, rowid:int=None):
        """Retrieve a single row by its internal SQLite rowid."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or rowid is None: return None
        return dbh.execute("SELECT * FROM {0} WHERE rowid = {1}".format(__class__.tablename, rowid)).fetchone()

    @staticmethod
    def fetch_all(dbh:DatabaseHandler):
        """Retrieve all robot rows."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): 
            return []
        return dbh.execute("SELECT * FROM {0}".format(__class__.tablename)).fetchall()

    @staticmethod
    def print_all(dbh:DatabaseHandler):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def insert_one(dbh:DatabaseHandler, robot_id:str=None, ipv4:str=None, descshort:str="", desclong:str=""):
        """Insert or replace a single robot entry."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or robot_id is None:
            return None
        dbh.execute(
            "INSERT OR REPLACE INTO {0}(robot_id, ipv4, descshort, desclong) VALUES(?, ?, ?, ?)".format(__class__.tablename),
            (robot_id, ipv4, descshort, desclong)
        )
    
    @staticmethod
    def default_insert(dbh:DatabaseHandler):
        """Insert default/seed data into the table (ROS 2 compatible robot_ids)."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        # ROS 2 naming schema: ur_1, panda_1, portal (no port specifications in robot_id)
        dbh.execute("INSERT OR IGNORE INTO {0}(robot_id, ipv4, descshort, desclong) VALUES('ur_1', '172.31.52.50', 'UR10e', 'Universal Robots 10 e-Serie (Portal-mounted)')".format(__class__.tablename) )
        dbh.execute("INSERT OR IGNORE INTO {0}(robot_id, ipv4, descshort, desclong) VALUES('panda_1', '172.31.52.60', 'Franka Panda', 'Franka Emika Robot Panda (future)')".format(__class__.tablename) )
        dbh.execute("INSERT OR IGNORE INTO {0}(robot_id, ipv4, descshort, desclong) VALUES('portal', '172.31.52.5', 'Portal XY', 'Sojka Portal XY Linear Positioning System')".format(__class__.tablename) )

class DBObj_ActionType:
    """Helper methods for managing the `ActionType` taxonomy.

    Action types categorize higher-level behaviours (e.g. `Trajectory`,
    `Gripper`). The helper exposes idempotent schema routines and boilerplate
    queries so the API layer can keep type catalogues synchronized.
    """
    tablename : str = "ActionType"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid         INTEGER PRIMARY KEY,"
                " actiontype_id TEXT    NOT NULL UNIQUE,"
                " desc          TEXT    UNIQUE"
                ")"
                ).format(__class__.tablename)
        dbh.execute(sqlquery)
    
    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def fetch_one(dbh:DatabaseHandler, actiontype_id:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute("SELECT * FROM {0} WHERE actiontype_id = '{1}'".format(__class__.tablename, actiontype_id)).fetchone()
    
    @staticmethod
    def fetch_one_by_row_id(dbh:DatabaseHandler, rowid:int=None):
        """Retrieve a single row by its internal SQLite rowid."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or rowid is None: return None
        return dbh.execute("SELECT * FROM {0} WHERE rowid = {1}".format(__class__.tablename, rowid)).fetchone()

    @staticmethod
    def print_all(dbh:DatabaseHandler):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def insert_one(dbh:DatabaseHandler, actiontype_id:str=None, desc:str=""):
        """Insert or replace a single action type entry."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or actiontype_id is None:
            return None
        dbh.execute(
            "INSERT OR REPLACE INTO {0}(actiontype_id, desc) VALUES(?, ?)".format(__class__.tablename),
            (actiontype_id, desc)
        )
    
    @staticmethod
    def default_insert(dbh:DatabaseHandler):
        """Insert default/seed data into the table."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("INSERT OR IGNORE INTO {0}(actiontype_id, desc) VALUES('MoveToConfig', 'move to a specific configuration')".format(__class__.tablename) )
        dbh.execute("INSERT OR IGNORE INTO {0}(actiontype_id, desc) VALUES('Trajectory', 'move along a trajectory')".format(__class__.tablename) )
        dbh.execute("INSERT OR IGNORE INTO {0}(actiontype_id, desc) VALUES('Gripper', 'gripper actions')".format(__class__.tablename) )

class DBObj_GripperType:
    """Helper methods for managing the `GripperType` taxonomy.
    
    Gripper types define physical specifications and capabilities for different
    gripper models (e.g. RG6, RGx). Stores min/max limits for width, force, 
    load, and feature flags like depth_compensation support.
    
    This table enables dynamic gripper parameter mapping and validation without
    hardcoding limits in application logic.
    """
    tablename: str = "GripperType"
    
    @staticmethod
    def create_tables(dbh: DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        sqlquery = (
            "CREATE TABLE IF NOT EXISTS {0}"
            "("
            " rowid                  INTEGER PRIMARY KEY,"
            " grippertype_id         TEXT    NOT NULL UNIQUE,"
            " desc                   TEXT,"
            " min_width_m            REAL    NOT NULL,"  # Minimum width in meters
            " max_width_m            REAL    NOT NULL,"  # Maximum width in meters
            " min_force_n            REAL    NOT NULL,"  # Minimum force in Newtons
            " max_force_n            REAL    NOT NULL,"  # Maximum force in Newtons
            " min_load_kg            REAL    NOT NULL,"  # Minimum load in kilograms
            " max_load_kg            REAL    NOT NULL,"  # Maximum load in kilograms
            " depth_compensation     INTEGER NOT NULL DEFAULT 0"  # 0=False, 1=True
            ")"
        ).format(__class__.tablename)
        dbh.execute(sqlquery)
    
    @staticmethod
    def drop_tables(dbh: DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))
    
    @staticmethod
    def schema_tables(dbh: DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def fetch_one(dbh: DatabaseHandler, grippertype_id: str = ""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        return dbh.execute(
            "SELECT * FROM {0} WHERE grippertype_id = '{1}'".format(__class__.tablename, grippertype_id)
        ).fetchone()
    
    @staticmethod
    def fetch_one_by_row_id(dbh: DatabaseHandler, rowid: int = None):
        """Retrieve a single row by its internal SQLite rowid."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or rowid is None:
            return None
        return dbh.execute(
            "SELECT * FROM {0} WHERE rowid = {1}".format(__class__.tablename, rowid)
        ).fetchone()
    
    @staticmethod
    def fetch_all(dbh: DatabaseHandler):
        """Retrieve all gripper types."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return []
        return dbh.execute("SELECT * FROM {0}".format(__class__.tablename)).fetchall()
    
    @staticmethod
    def print_all(dbh: DatabaseHandler):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description]
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def insert_one(
        dbh: DatabaseHandler,
        grippertype_id: str = None,
        desc: str = "",
        min_width_m: float = 0.0,
        max_width_m: float = 0.0,
        min_force_n: float = 0.0,
        max_force_n: float = 0.0,
        min_load_kg: float = 0.0,
        max_load_kg: float = 0.0,
        depth_compensation: bool = False
    ):
        """Insert or replace a single gripper type entry."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or grippertype_id is None:
            return None
        dbh.execute(
            "INSERT OR REPLACE INTO {0}"
            "(grippertype_id, desc, min_width_m, max_width_m, min_force_n, max_force_n, "
            "min_load_kg, max_load_kg, depth_compensation) "
            "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)".format(__class__.tablename),
            (grippertype_id, desc, min_width_m, max_width_m, min_force_n, max_force_n,
             min_load_kg, max_load_kg, 1 if depth_compensation else 0)
        )
    
    @staticmethod
    def default_insert(dbh: DatabaseHandler):
        """Insert default/seed data into the table.
        
        Initial data includes RG6 gripper specifications:
        - Width: 0.0 - 0.120m (0-120mm)
        - Force: 5 - 80N
        - Load: 0.0 - 5.0kg
        - Depth compensation: Supported
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        dbh.execute(
            "INSERT OR IGNORE INTO {0}"
            "(grippertype_id, desc, min_width_m, max_width_m, min_force_n, max_force_n, "
            "min_load_kg, max_load_kg, depth_compensation) "
            "VALUES('RG6', 'OnRobot RG6 Gripper', 0.0, 0.120, 5.0, 80.0, 0.0, 5.0, 1)".format(__class__.tablename)
        )

class DBObj_Action:
    """Manage the `Action` table that stores high-level motion metadata.

    Each action references an `ActionType`, target robot, and a collection of
    derived metadata that is synchronized with waypoint data. The methods here
    support schema management, CRUD routines, and metadata backfilling used by
    higher-level planning APIs.
    """
    tablename : str = "Action"
    _METADATA_COLUMNS: Tuple[Tuple[str, str], ...] = (
        ("format_version", "TEXT"),
        ("coordinate_system", "TEXT"),
        ("creation_time_ms", "INTEGER"),
        ("duration_ms", "INTEGER"),
        ("waypoint_count", "INTEGER"),
        ("count_dof", "INTEGER"),
        ("velocity_scaling", "REAL"),
        ("acceleration_scaling", "REAL"),
        ("metadata_json", "TEXT"),
    )

    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        if not DBObj_Action._table_exists(dbh):
            DBObj_Action._create_table(dbh)
        else:
            existing_columns = DBObj_Action._existing_columns(dbh)
            if "actiontime" in existing_columns:
                DBObj_Action._migrate_drop_actiontime(dbh)
        DBObj_Action._ensure_metadata_columns(dbh)

    @staticmethod
    def insert_one(dbh:DatabaseHandler, action_id:str=None, actiontype_id:str=None, robot_id:str=None, metadata: Optional[Dict[str, Any]] = None):
        """Insert or replace a single row."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or action_id is None or actiontype_id is None or robot_id is None: return None

        actiontype_row = dbh.execute(
            "SELECT rowid FROM {0} WHERE actiontype_id = ?".format(DBObj_ActionType.tablename),
            (actiontype_id,),
        ).fetchone()
        robot_row = dbh.execute(
            "SELECT rowid FROM {0} WHERE robot_id = ?".format(DBObj_Robot.tablename),
            (robot_id,),
        ).fetchone()
        if actiontype_row is None or robot_row is None:
            return None

        dbh.execute(
            "INSERT OR REPLACE INTO {0}(action_id, actiontype_rowid, robot_rowid) VALUES (?, ?, ?)".format(__class__.tablename),
            (action_id, actiontype_row[0], robot_row[0]),
        )
        if metadata:
            DBObj_Action._update_metadata(dbh, action_id, metadata)

    @staticmethod
    def rename_one(dbh:DatabaseHandler, action_id:str=None, action_id_renamed:str=None):
        """Rename an existing row identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or action_id is None or action_id_renamed is None: return None
        dbh.execute("UPDATE {0} SET action_id = '{1}' WHERE action_id = '{2}'".format(__class__.tablename, action_id_renamed, action_id))

    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))

    @staticmethod
    def fetch_one(dbh:DatabaseHandler, action_id:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute("SELECT * FROM {0} WHERE action_id = '{1}'".format(__class__.tablename, action_id)).fetchone()
    
    @staticmethod
    def fetch_one_by_row_id(dbh:DatabaseHandler, rowid:int=None):
        """Retrieve a single row by its internal SQLite rowid."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or rowid is None: return None
        return dbh.execute("SELECT * FROM {0} WHERE rowid = {1}".format(__class__.tablename, rowid)).fetchone()
    
    @staticmethod
    def fetch_all(dbh:DatabaseHandler, action_id:str=""):
        """Retrieve all rows, optionally filtered by identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(action_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE action_id = '{1}'".format(__class__.tablename, action_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()

        return list(rows)
    
    @staticmethod
    def delete_one(dbh:DatabaseHandler, action_id:str=""):
        """Delete a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        # due to table ActionDataPoint's foreign key constraint ON DELETE CASCADE: 
        # the delete action triggers a cascading deletion, i.e. all associated ActionDataPoint entries are also deleted
        dbh.execute("DELETE FROM {0} WHERE action_id = '{1}'".format(__class__.tablename, action_id))
    
    @staticmethod
    def count_usages(dbh:DatabaseHandler, action_id:str="") -> int:
        """Count how many activities use this action.
        
        Args:
            dbh: DatabaseHandler instance
            action_id: Action identifier to check
            
        Returns:
            Number of activities using this action
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler): return 0
        action_row = DBObj_Action.fetch_one(dbh, action_id)
        if action_row is None:
            return 0
        action_rowid = action_row["rowid"]
        result = dbh.execute(
            f"SELECT COUNT(*) as count FROM {DBObj_ActivityActionRel.tablename} WHERE action_rowid_act = ?",
            (action_rowid,)
        ).fetchone()
        return result["count"] if result else 0
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def backfill_metadata(dbh: DatabaseHandler):
        """Recompute derived metadata fields from waypoint data.
        
        Calculates duration_ms, waypoint_count, and count_dof from related
        ActionDataPoint and ActionDataPointCoords entries.
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        dbh.execute(
            f"""
            UPDATE {DBObj_Action.tablename} SET
              duration_ms = (
                SELECT COALESCE(MAX(t_ms) - MIN(t_ms), 0)
                FROM ActionDataPoint adp
                WHERE adp.action_rowid = {DBObj_Action.tablename}.rowid
              ),
              waypoint_count = (
                SELECT COUNT(1)
                FROM ActionDataPoint adp
                WHERE adp.action_rowid = {DBObj_Action.tablename}.rowid
              )
            WHERE duration_ms IS NULL OR waypoint_count IS NULL
            """
        )

        dbh.execute(
            f"""
            UPDATE {DBObj_Action.tablename} SET
              count_dof = (
                SELECT COALESCE(MAX(j.coord_index) + 1, 0)
                FROM ActionDataPointCoords j
                JOIN ActionDataPoint wp ON wp.rowid = j.datapoint_rowid
                WHERE wp.action_rowid = {DBObj_Action.tablename}.rowid
              )
            WHERE count_dof IS NULL
            """
        )

    @staticmethod
    def print_all(dbh:DatabaseHandler, action_id:str=""):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(action_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE action_id = '{1}'".format(__class__.tablename, action_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def _existing_columns(dbh: DatabaseHandler, table_name: Optional[str] = None) -> List[str]:
        """Return available column names for `table_name` (defaults to `Action`)."""
        target_table = table_name or __class__.tablename
        retcur = dbh.execute("pragma table_info('{0}')".format(target_table))
        rows = retcur.fetchall()
        if not rows:
            return []
        if isinstance(rows[0], sqlite3.Row):
            return [row["name"] for row in rows]
        return [row[1] for row in rows]

    @staticmethod
    def _table_exists(dbh: DatabaseHandler) -> bool:
        """Check whether the `Action` table is already present."""
        retcur = dbh.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
            (__class__.tablename,),
        )
        return retcur.fetchone() is not None

    @staticmethod
    def _create_table(dbh: DatabaseHandler) -> None:
        """Create the modern `Action` table structure."""
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid             INTEGER     PRIMARY KEY,"
                " action_id         TEXT        NOT NULL UNIQUE,"
                " actiontype_rowid  INTEGER     NOT NULL,"
                " robot_rowid       INTEGER     NOT NULL,"
                " format_version    TEXT,"
                " coordinate_system TEXT,"
                " creation_time_ms  INTEGER,"
                " duration_ms       INTEGER,"
                " waypoint_count    INTEGER,"
                " count_dof         INTEGER,"
                " velocity_scaling  REAL,"
                " acceleration_scaling REAL,"
                " metadata_json     TEXT,"
                " FOREIGN KEY (actiontype_rowid)"
                " REFERENCES {1} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE RESTRICT"
                " FOREIGN KEY (robot_rowid)"
                " REFERENCES {2} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE RESTRICT"
                ")"
                ).format(__class__.tablename, DBObj_ActionType.tablename, DBObj_Robot.tablename)
        dbh.execute(sqlquery)

    @staticmethod
    def _migrate_drop_actiontime(dbh: DatabaseHandler) -> None:
        """Rebuild legacy tables that still contain the obsolete `actiontime` column."""
        legacy_table = f"{__class__.tablename}_legacy"
        try:
            dbh.commit()
        except Exception:
            pass
        dbh.execute("BEGIN IMMEDIATE")
        try:
            dbh.execute(
                f"ALTER TABLE {__class__.tablename} RENAME TO {legacy_table}"
            )
            DBObj_Action._create_table(dbh)

            legacy_columns = set(DBObj_Action._existing_columns(dbh, legacy_table))
            copy_columns: List[str] = ["rowid", "action_id", "actiontype_rowid", "robot_rowid"]
            for column_name, _ in DBObj_Action._METADATA_COLUMNS:
                if column_name in legacy_columns and column_name not in copy_columns:
                    copy_columns.append(column_name)

            column_list = ", ".join(copy_columns)
            dbh.execute(
                f"INSERT INTO {__class__.tablename} ({column_list}) "
                f"SELECT {column_list} FROM {legacy_table}"
            )

            dbh.execute(f"DROP TABLE IF EXISTS {legacy_table}")
            dbh.commit()
        except Exception:
            dbh.rollback()
            raise

    @staticmethod
    def _ensure_metadata_columns(dbh: DatabaseHandler):
        """Add any missing metadata columns to the `Action` table."""
        try:
            existing = DBObj_Action._existing_columns(dbh)
        except Exception:
            return
        for column_name, column_type in DBObj_Action._METADATA_COLUMNS:
            if column_name not in existing:
                dbh.execute(
                    f"ALTER TABLE {DBObj_Action.tablename} ADD COLUMN {column_name} {column_type}"
                )

    @staticmethod
    def _update_metadata(dbh: DatabaseHandler, action_id: str, metadata: Dict[str, Any]):
        """Persist structured metadata for an action.

        Args:
            dbh (DatabaseHandler): Open database handle.
            action_id (str): Target action identifier to update.
            metadata (dict): Metadata payload received from the API layer.
        """
        allowed_keys = {name for name, _ in DBObj_Action._METADATA_COLUMNS if name != "metadata_json"}
        set_clauses: List[str] = []
        parameters: List[Any] = []
        metadata_overflow: Dict[str, Any] = {}
        for key, value in metadata.items():
            if value is None:
                continue
            if key in allowed_keys:
                set_clauses.append(f"{key} = ?")
                parameters.append(value)
            elif key == "metadata_json":
                set_clauses.append("metadata_json = ?")
                parameters.append(value if isinstance(value, str) else json.dumps(value))
            else:
                metadata_overflow[key] = value
        if metadata_overflow:
            set_clauses.append("metadata_json = ?")
            parameters.append(json.dumps(metadata_overflow))
        if not set_clauses:
            return
        parameters.append(action_id)
        dbh.execute(
            f"UPDATE {DBObj_Action.tablename} SET {', '.join(set_clauses)} WHERE action_id = ?",
            tuple(parameters),
        )

class DBObj_ActionDataPoint:
    """Persist normalized waypoint samples tied to an action.

    The table contains sequence-ordered waypoints, optional gripper telemetry,
    and references to coordinate values stored in :class:`DBObj_ActionDataPointCoords`.
    Helpers manage schema migrations and conversions from legacy formats so the
    higher-level API can always insert clean time-series data.
    """
    tablename: str = "ActionDataPoint"
    legacy_tablename: str = "ActionDataPoint_legacy"
    _DEG_TO_RAD: float = math.pi / 180.0
    _INDEX_STATEMENTS: Tuple[str, ...] = (
        "CREATE INDEX IF NOT EXISTS idx_adp_action_seq ON ActionDataPoint(action_rowid, seq_no)",
        "CREATE INDEX IF NOT EXISTS idx_adp_action_time ON ActionDataPoint(action_rowid, t_ms)",
    )

    @staticmethod
    def create_tables(dbh: DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        if not DBObj_ActionDataPoint._table_exists(dbh):
            DBObj_ActionDataPoint._create_new_table(dbh)
        elif DBObj_ActionDataPoint._is_legacy_schema(dbh):
            DBObj_ActionDataPoint._migrate_legacy_schema(dbh)
        else:
            DBObj_ActionDataPoint._ensure_indexes(dbh)

        DBObj_ActionDataPointCoords.create_tables(dbh)
        DBObj_Action.backfill_metadata(dbh)

    @staticmethod
    def insert_by_action_id(
        dbh: DatabaseHandler,
        action_id: Optional[str] = None,
        waypoints: Optional[Sequence[Dict[str, Any]]] = None,
    ):
        """Replace all waypoints for an action.
        
        Deletes existing waypoints and inserts new ones from the provided sequence.
        Also updates metadata like duration and waypoint count.
        
        Args:
            dbh: Open database handle
            action_id: External action identifier
            waypoints: Sequence of waypoint dictionaries, each containing:
                - seq_no (int, optional): Sequence number (defaults to index)
                - t_ms (int, optional): Time in milliseconds (or use 'time_ms', 'time', 't')
                - generalized_coords (list[float], optional): Generalized coordinates (radians for joints, meters for cartesian)
                  (can also use 'joint_values', 'joints', or 'q')
                - gripper_width (float, optional): Gripper width
                - gripper_force (float, optional): Gripper force
                - gripper_load (float, optional): Gripper load
        
        Example:
            waypoints = [
                {"seq_no": 0, "t_ms": 0, "generalized_coords": [0.0, 0.5], "gripper_width": 0.08},
                {"seq_no": 1, "t_ms": 120, "generalized_coords": [0.1, 0.6], "gripper_force": 25.0}
            ]
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or action_id is None:
            return None

        action_row = dbh.execute(
            f"SELECT rowid FROM {DBObj_Action.tablename} WHERE action_id = ?",
            (action_id,),
        ).fetchone()
        if action_row is None:
            return None
        action_rowid = action_row[0]

        DBObj_ActionDataPoint._delete_by_action_rowid(dbh, action_rowid)
        if not waypoints:
            DBObj_Action.backfill_metadata(dbh)
            return None

        DBObj_ActionDataPoint._ensure_indexes(dbh)
        DBObj_ActionDataPointCoords.create_tables(dbh)

        insert_wp_sql = (
            f"INSERT INTO {DBObj_ActionDataPoint.tablename} "
            "(action_rowid, seq_no, t_ms, gripper_width, gripper_force, gripper_load) "
            "VALUES (?, ?, ?, ?, ?, ?)"
        )
        insert_coord_sql = (
            f"INSERT INTO {DBObj_ActionDataPointCoords.tablename} "
            "(datapoint_rowid, coord_index, coord_value) VALUES (?, ?, ?)"
        )

        for default_seq, payload in enumerate(waypoints):
            seq_no, t_ms, gripper = DBObj_ActionDataPoint._normalize_waypoint(payload, default_seq)
            dbh.execute(insert_wp_sql, (action_rowid, seq_no, t_ms, gripper[0], gripper[1], gripper[2]))
            datapoint_rowid = dbh.execute("SELECT last_insert_rowid()").fetchone()[0]
            for coord_index, coord_value in DBObj_ActionDataPoint._iter_coord_entries(payload):
                dbh.execute(insert_coord_sql, (datapoint_rowid, coord_index, coord_value))

        DBObj_Action.backfill_metadata(dbh)

    @staticmethod
    def drop_tables(dbh: DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        DBObj_ActionDataPointCoords.drop_tables(dbh)
        dbh.execute(f"DROP TABLE IF EXISTS {DBObj_ActionDataPoint.tablename}")

    @staticmethod
    def schema_tables(dbh: DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        retcur = dbh.execute(
            f"SELECT sql FROM sqlite_schema WHERE name = '{DBObj_ActionDataPoint.tablename}'"
        )
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute(
            f"pragma table_info('{DBObj_ActionDataPoint.tablename}')"
        )
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def print_all(dbh: DatabaseHandler, action_rowid: Optional[int] = None):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or action_rowid is None:
            return None
        rows = DBObj_ActionDataPoint.fetch_all_by_action_row_id(dbh, action_rowid)
        print(f"table {DBObj_ActionDataPoint.tablename}:")
        for row in rows:
            print(row)
        print()

    @staticmethod
    def fetch_all_by_action_row_id(dbh: DatabaseHandler, action_rowid: Optional[int] = None):
        """Return waypoints with embedded joint values for an action.
        
        Merges waypoint and joint data into nested dictionaries.
        
        Args:
            dbh: Open database handle
            action_rowid: Action table row ID
        
        Returns:
            List of dictionaries, each containing:
                - rowid (int): Waypoint row ID
                - seq_no (int): Sequence number
                - t_ms (int): Time in milliseconds
                - gripper_width (float, optional): Gripper width
                - gripper_force (float, optional): Gripper force
                - gripper_load (float, optional): Gripper load
                - generalized_coords (list[float]): Generalized coordinates (radians for joints, meters for cartesian - sparse list, trailing None removed)
        
        Example output:
            [
                {"rowid": 1, "seq_no": 0, "t_ms": 0, "gripper_width": 0.08, 
                 "gripper_force": None, "gripper_load": None, "generalized_coords": [0.0, 0.5, 0.2]},
                {"rowid": 2, "seq_no": 1, "t_ms": 120, "gripper_width": 0.079,
                 "gripper_force": 25.0, "gripper_load": None, "generalized_coords": [0.1, 0.6, 0.25]}
            ]
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or action_rowid is None:
            return None
        retcur = dbh.execute(
            f"""
            SELECT wp.rowid, wp.seq_no, wp.t_ms, wp.gripper_width, wp.gripper_force, wp.gripper_load,
                   j.coord_index, j.coord_value
            FROM {DBObj_ActionDataPoint.tablename} wp
            LEFT JOIN {DBObj_ActionDataPointCoords.tablename} j
              ON j.datapoint_rowid = wp.rowid
            WHERE wp.action_rowid = ?
            ORDER BY wp.seq_no ASC, j.coord_index ASC
            """,
            (action_rowid,),
        )
        rows = retcur.fetchall()
        return DBObj_ActionDataPoint._merge_joint_rows(rows)

    @staticmethod
    def _merge_joint_rows(rows: Iterable[sqlite3.Row]) -> List[Dict[str, Any]]:
        """Combine waypoint and coordinate result rows into nested dictionaries."""
        merged: List[Dict[str, Any]] = []
        current: Optional[Dict[str, Any]] = None
        for row in rows:
            if current is None or current["rowid"] != row["rowid"]:
                if current is not None:
                    current["generalized_coords"] = DBObj_ActionDataPoint._compact_joint_list(current.get("generalized_coords", []))
                    merged.append(current)
                current = {
                    "rowid": row["rowid"],
                    "seq_no": row["seq_no"],
                    "t_ms": row["t_ms"],
                    "gripper_width": row["gripper_width"],
                    "gripper_force": row["gripper_force"],
                    "gripper_load": row["gripper_load"],
                    "generalized_coords": [],
                }
            if row["coord_index"] is not None:
                DBObj_ActionDataPoint._assign_joint_value(current["generalized_coords"], row["coord_index"], row["coord_value"])
        if current is not None:
            current["generalized_coords"] = DBObj_ActionDataPoint._compact_joint_list(current.get("generalized_coords", []))
            merged.append(current)
        return merged

    @staticmethod
    def _compact_joint_list(values: List[Optional[float]]) -> List[Optional[float]]:
        """Trim trailing ``None`` entries to keep joint arrays compact."""
        while values and values[-1] is None:
            values.pop()
        return values

    @staticmethod
    def _assign_joint_value(target: List[Optional[float]], index: int, value: float) -> None:
        """Assign a joint value into ``target`` expanding the list as needed."""
        while len(target) <= index:
            target.append(None)
        target[index] = value

    @staticmethod
    def _normalize_waypoint(payload: Dict[str, Any], default_seq: int) -> Tuple[int, int, Tuple[Optional[float], Optional[float], Optional[float]]]:
        """Map raw waypoint payload into normalized tuple (seq, time, gripper data)."""
        seq_no = payload.get("seq_no")
        if seq_no is None:
            seq_no = payload.get("index", default_seq)
        try:
            seq_no = int(seq_no)
        except (TypeError, ValueError):
            seq_no = default_seq

        t_ms = payload.get("t_ms")
        if t_ms is None:
            t_ms = payload.get("time_ms")
        if t_ms is None:
            seconds = payload.get("time") or payload.get("t")
            t_ms = int(round(float(seconds) * 1000.0)) if seconds is not None else 0
        else:
            t_ms = int(round(float(t_ms)))

        gripper_width = payload.get("gripper_width")
        gripper_force = payload.get("gripper_force")
        gripper_load = payload.get("gripper_load")
        return seq_no, t_ms, (gripper_width, gripper_force, gripper_load)

    @staticmethod
    def _iter_coord_entries(payload: Dict[str, Any]) -> List[Tuple[int, float]]:
        """Yield coordinate index/value pairs found in a waypoint payload."""
        joint_container = None
        for key in ("generalized_coords", "joint_values", "joints", "q"):
            if key in payload and payload[key] is not None:
                joint_container = payload[key]
                break
        if joint_container is None:
            return []

        entries: List[Tuple[int, float]] = []
        if isinstance(joint_container, dict):
            iterable = joint_container.items()
        else:
            iterable = enumerate(joint_container)
        for j_index, value in iterable:
            if value is None:
                continue
            try:
                idx_int = int(j_index)
                val_float = float(value)
            except (TypeError, ValueError):
                continue
            entries.append((idx_int, val_float))
        entries.sort(key=lambda item: item[0])
        return entries

    @staticmethod
    def _delete_by_action_rowid(dbh: DatabaseHandler, action_rowid: int):
        """Delete all waypoint rows bound to ``action_rowid``."""
        dbh.execute(
            f"DELETE FROM {DBObj_ActionDataPoint.tablename} WHERE action_rowid = ?",
            (action_rowid,),
        )

    @staticmethod
    def _table_exists(dbh: DatabaseHandler) -> bool:
        """Return ``True`` if the waypoint table already exists."""
        retcur = dbh.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
            (DBObj_ActionDataPoint.tablename,),
        )
        return retcur.fetchone() is not None

    @staticmethod
    def _fetch_columns(dbh: DatabaseHandler) -> List[str]:
        """Fetch column names from the current waypoint table."""
        retcur = dbh.execute(
            f"pragma table_info('{DBObj_ActionDataPoint.tablename}')"
        )
        rows = retcur.fetchall()
        if not rows:
            return []
        if isinstance(rows[0], sqlite3.Row):
            return [row["name"] for row in rows]
        return [row[1] for row in rows]

    @staticmethod
    def _is_legacy_schema(dbh: DatabaseHandler) -> bool:
        """Detect pre-upgrade schemas using ``time`` instead of ``t_ms``."""
        columns = DBObj_ActionDataPoint._fetch_columns(dbh)
        return "time" in columns and "t_ms" not in columns

    @staticmethod
    def _create_new_table(dbh: DatabaseHandler):
        """Create the current waypoint table definition if missing."""
        dbh.execute(
            f"""
            CREATE TABLE IF NOT EXISTS {DBObj_ActionDataPoint.tablename} (
                rowid INTEGER PRIMARY KEY,
                action_rowid INTEGER NOT NULL
                    REFERENCES {DBObj_Action.tablename}(rowid) ON DELETE CASCADE,
                seq_no INTEGER NOT NULL,
                t_ms INTEGER NOT NULL,
                gripper_width REAL,
                gripper_force REAL,
                gripper_load REAL,
                UNIQUE(action_rowid, seq_no)
            )
            """
        )
        DBObj_ActionDataPoint._ensure_indexes(dbh)

    @staticmethod
    def _ensure_indexes(dbh: DatabaseHandler):
        """Create the sequence/time indexes used by deterministic queries."""
        for statement in DBObj_ActionDataPoint._INDEX_STATEMENTS:
            dbh.execute(statement)

    @staticmethod
    def _migrate_legacy_schema(dbh: DatabaseHandler):
        """Upgrade legacy waypoint tables to the normalized structure."""
        try:
            dbh.commit()
        except Exception:
            pass
        dbh.execute("BEGIN IMMEDIATE")
        try:
            dbh.execute(
                f"ALTER TABLE {DBObj_ActionDataPoint.tablename} RENAME TO {DBObj_ActionDataPoint.legacy_tablename}"
            )
            DBObj_ActionDataPoint._create_new_table(dbh)
            DBObj_ActionDataPointCoords.create_tables(dbh)

            legacy_rows = dbh.execute(
                f"SELECT * FROM {DBObj_ActionDataPoint.legacy_tablename} ORDER BY action_rowid, time, rowid"
            ).fetchall()

            insert_wp_sql = (
                f"INSERT INTO {DBObj_ActionDataPoint.tablename} "
                "(action_rowid, seq_no, t_ms, gripper_width, gripper_force, gripper_load) "
                "VALUES (?, ?, ?, ?, ?, ?)"
            )
            insert_coord_sql = (
                f"INSERT INTO {DBObj_ActionDataPointCoords.tablename} "
                "(datapoint_rowid, coord_index, coord_value) VALUES (?, ?, ?)"
            )

            seq_counter: Dict[int, int] = {}
            for row in legacy_rows:
                action_rowid = row["action_rowid"]
                seq_no = seq_counter.get(action_rowid, 0)
                seq_counter[action_rowid] = seq_no + 1
                dbh.execute(
                    insert_wp_sql,
                    (
                        action_rowid,
                        seq_no,
                        row["time"],
                        row["gripper_width"],
                        row["gripper_force"],
                        row["gripper_load"],
                    ),
                )
                datapoint_rowid = dbh.execute("SELECT last_insert_rowid()").fetchone()[0]

                for coord_index in range(10):
                    key = f"q{coord_index}"
                    value = row[key]
                    if value is None:
                        continue
                    dbh.execute(
                        insert_coord_sql,
                        (
                            datapoint_rowid,
                            coord_index,
                            float(value) * DBObj_ActionDataPoint._DEG_TO_RAD,
                        ),
                    )

            dbh.execute(
                f"DROP TABLE IF EXISTS {DBObj_ActionDataPoint.legacy_tablename}"
            )
            dbh.commit()
        except Exception:
            dbh.rollback()
            raise

class DBObj_ActionDataPointCoords:
    """Store generalized coordinate values associated with each waypoint.
    
    Stores coordinate values (joint angles in radians, cartesian positions in meters)
    for each waypoint in a normalized format. The interpretation depends on the
    Action's coordinate_system metadata field.
    """
    tablename: str = "ActionDataPointCoords"

    @staticmethod
    def create_tables(dbh: DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        dbh.execute(
            f"""
            CREATE TABLE IF NOT EXISTS {DBObj_ActionDataPointCoords.tablename} (
                rowid INTEGER PRIMARY KEY,
                datapoint_rowid INTEGER NOT NULL
                    REFERENCES {DBObj_ActionDataPoint.tablename}(rowid) ON DELETE CASCADE,
                coord_index INTEGER NOT NULL,
                coord_value REAL NOT NULL,
                UNIQUE(datapoint_rowid, coord_index)
            )
            """
        )
        dbh.execute(
            f"CREATE INDEX IF NOT EXISTS idx_adpc_dp ON {DBObj_ActionDataPointCoords.tablename}(datapoint_rowid, coord_index)"
        )

    @staticmethod
    def drop_tables(dbh: DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        dbh.execute(
            f"DROP TABLE IF EXISTS {DBObj_ActionDataPointCoords.tablename}"
        )

    @staticmethod
    def schema_tables(dbh: DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return None
        retcur = dbh.execute(
            f"SELECT sql FROM sqlite_schema WHERE name = '{DBObj_ActionDataPointCoords.tablename}'"
        )
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute(
            f"pragma table_info('{DBObj_ActionDataPointCoords.tablename}')"
        )
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def print_all(dbh: DatabaseHandler, datapoint_rowid: Optional[int] = None):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or datapoint_rowid is None:
            return None
        retcur = dbh.execute(
            f"SELECT * FROM {DBObj_ActionDataPointCoords.tablename} WHERE datapoint_rowid = ?",
            (datapoint_rowid,),
        )
        cols = [description[0] for description in retcur.description]
        rows = retcur.fetchall()
        print(f"table {DBObj_ActionDataPointCoords.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def fetch_by_datapoint_row_id(dbh: DatabaseHandler, datapoint_rowid: Optional[int] = None):
        """Fetch all coordinate values for a waypoint identifier.
        
        Args:
            dbh: Open database handle
            datapoint_rowid: Waypoint row ID
        
        Returns:
            List of sqlite3.Row objects, each containing:
                - rowid (int): Coordinate entry row ID
                - datapoint_rowid (int): Foreign key to ActionDataPoint table
                - coord_index (int): Coordinate index (0-based)
                - coord_value (float): Coordinate value (radians for joints, meters for cartesian)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or datapoint_rowid is None:
            return None
        return dbh.execute(
            f"SELECT * FROM {DBObj_ActionDataPointCoords.tablename} WHERE datapoint_rowid = ?",
            (datapoint_rowid,),
        ).fetchall()


class DBObj_Activity:
    """Represent high-level activities composed of ordered actions."""
    tablename : str = "Activity"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid             INTEGER     PRIMARY KEY,"
                " activity_id       TEXT        NOT NULL UNIQUE,"
                " activitytime      INTEGER     NOT NULL"
                ")"
                ).format(__class__.tablename)
        dbh.execute(sqlquery)

    @staticmethod
    def insert_one(dbh:DatabaseHandler, activity_id:str=None, activitytime:int=0):
        """Insert or replace a single row."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None: return None
        # INSERT OR REPLACE: add a new one or update an existing
        sqlquery = (
                "INSERT OR REPLACE INTO {0}"
                "(activity_id, activitytime) VALUES "
                "('{1}', {2})"
            ).format(__class__.tablename, activity_id, activitytime)
        dbh.execute(sqlquery)

    @staticmethod
    def rename_one(dbh:DatabaseHandler, activity_id:str=None, activity_id_renamed:str=None):
        """Rename an existing row identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None or activity_id_renamed is None: return None
        dbh.execute("UPDATE TABLE {0} SET activity_id = '{1}' WHERE activity_id = '{2}'".format(__class__.tablename, activity_id_renamed, activity_id))

    @staticmethod
    def update_time(dbh:DatabaseHandler, activity_id:str=None, activitytime:int=0):
        """Update the activity time for an existing row identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None: return None
        dbh.execute("UPDATE {0} SET activitytime = {1} WHERE activity_id = '{2}'".format(__class__.tablename, activitytime, activity_id))

    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))

    @staticmethod
    def fetch_one(dbh:DatabaseHandler, activity_id:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute("SELECT * FROM {0} WHERE activity_id = '{1}'".format(__class__.tablename, activity_id)).fetchone()
    
    @staticmethod
    def fetch_one_by_row_id(dbh:DatabaseHandler, rowid:int=None):
        """Retrieve a single row by its internal SQLite rowid."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or rowid is None: return None
        return dbh.execute("SELECT * FROM {0} WHERE rowid = {1}".format(__class__.tablename, rowid)).fetchone()

    @staticmethod
    def delete_one(dbh:DatabaseHandler, activity_id:str=""):
        """Delete a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DELETE FROM {0} WHERE activity_id = '{1}'".format(__class__.tablename, activity_id))
    
    @staticmethod
    def count_usages(dbh:DatabaseHandler, activity_id:str="") -> int:
        """Count how many tasks use this activity.
        
        Args:
            dbh: DatabaseHandler instance
            activity_id: Activity identifier to check
            
        Returns:
            Number of tasks using this activity
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler): return 0
        activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
        if activity_row is None:
            return 0
        activity_rowid = activity_row["rowid"]
        result = dbh.execute(
            f"SELECT COUNT(*) as count FROM {DBObj_TaskActivityRel.tablename} WHERE activity_rowid = ?",
            (activity_rowid,)
        ).fetchone()
        return result["count"] if result else 0
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    

    @staticmethod
    def fetch_all(dbh:DatabaseHandler, activity_id:str=""):
        """Retrieve all rows, optionally filtered by identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(activity_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE activity_id = '{1}'".format(__class__.tablename, activity_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()

        return list(rows)

    @staticmethod
    def print_all(dbh:DatabaseHandler, activity_id:str=""):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(activity_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE activity_id = '{1}'".format(__class__.tablename, activity_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()

class DBObj_Task:
    """Model a task that aggregates multiple activities."""
    tablename : str = "Task"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid             INTEGER     PRIMARY KEY,"
                " task_id           TEXT        NOT NULL UNIQUE,"
                " tasktime          INTEGER     NOT NULL"
                ")"
                ).format(__class__.tablename)
        dbh.execute(sqlquery)

    @staticmethod
    def insert_one(dbh:DatabaseHandler, task_id:str=None, tasktime:int=0):
        """Insert or replace a single row."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or task_id is None: return None
        # INSERT OR REPLACE: add a new one or update an existing
        sqlquery = (
                "INSERT OR REPLACE INTO {0}"
                "(task_id, tasktime) VALUES "
                "('{1}', {2})"
            ).format(__class__.tablename, task_id, tasktime)
        dbh.execute(sqlquery)
    
    @staticmethod
    def rename_one(dbh:DatabaseHandler, task_id:str=None, task_id_renamed:str=None):
        """Rename an existing row identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or task_id is None or task_id_renamed is None: return None
        dbh.execute("UPDATE TABLE {0} SET task_id = '{1}' WHERE task_id = '{2}'".format(__class__.tablename, task_id_renamed, task_id))

    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))

    @staticmethod
    def fetch_one(dbh:DatabaseHandler, task_id:str=""):
        """Retrieve a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute("SELECT * FROM {0} WHERE task_id = '{1}'".format(__class__.tablename, task_id)).fetchone()
    
    @staticmethod
    def delete_one(dbh:DatabaseHandler, task_id:str=""):
        """Delete a single row by its identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DELETE FROM {0} WHERE task_id = '{1}'".format(__class__.tablename, task_id))
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def fetch_all(dbh:DatabaseHandler, task_id:str=""):
        """Retrieve all rows, optionally filtered by identifier."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(task_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE task_id = '{1}'".format(__class__.tablename, task_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()

        return rows

    @staticmethod
    def print_all(dbh:DatabaseHandler, task_id:str=""):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(task_id) > 0:
            sqlquery = "SELECT * FROM {0} WHERE task_id = '{1}'".format(__class__.tablename, task_id)
        else:
            sqlquery = "SELECT * FROM {0}".format(__class__.tablename)
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()

class DBObj_TaskActivityRel:
    """Manage the many-to-many relationship between tasks and activities."""
    tablename : str = "TaskActivityRel"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the table schema if it doesn't already exist."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid             INTEGER     PRIMARY KEY,"
                " task_rowid        INTEGER     NOT NULL,"
                " activity_rowid    INTEGER     NOT NULL,"
                " FOREIGN KEY (task_rowid)"
                " REFERENCES {1} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE CASCADE"
                " FOREIGN KEY (activity_rowid)"
                " REFERENCES {2} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE RESTRICT"
                ")"
                ).format(__class__.tablename, DBObj_Task.tablename, DBObj_Activity.tablename)
        dbh.execute(sqlquery)

    @staticmethod
    def insert_by_task(dbh:DatabaseHandler, task_id:str=None, listOfDict:list=None):
        """Replace all activity associations for a task.
        
        Deletes existing activity relations for the task and inserts new ones.
        
        Args:
            dbh: Open database handle
            task_id: Task identifier
            listOfDict: List of dictionaries, each containing:
                - activity_id (str): Activity identifier to associate with the task
        
        Example:
            listOfDict = [
                {"activity_id": "ACV001"},
                {"activity_id": "ACV002"}
            ]
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or task_id is None or listOfDict is None: return None

        # delete all related data first
        dbh.execute("DELETE FROM {0} WHERE task_rowid IN (SELECT rowid FROM {1} WHERE task_id = '{2}')".format(__class__.tablename, DBObj_Task.tablename, task_id))

        # insert new data
        sqlquery = (
                "INSERT INTO {0}"
                "( task_rowid, activity_rowid) VALUES "
                "((SELECT rowid FROM {1} WHERE task_id = '{3}'), (SELECT rowid FROM {2} WHERE activity_id = :activity_id))"
            ).format(__class__.tablename, DBObj_Task.tablename, DBObj_Activity.tablename, task_id)
        dbh.execute_many(sqlquery, listOfDict)

    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))

    @staticmethod
    def fetch_by_task(dbh:DatabaseHandler, task_id:str=""):
        """Fetch relation rows joined with identifiers for a task.
        
        Args:
            dbh: Open database handle
            task_id: Task identifier to filter by
        
        Returns:
            List of sqlite3.Row objects, each containing:
                - rowid (int): Relation table row ID
                - task_rowid (int): Foreign key to Task table
                - activity_rowid (int): Foreign key to Activity table
                - task_id (str): Task identifier (joined)
                - activity_id (str): Activity identifier (joined)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute(
            ("SELECT t0.*,t1.task_id, t2.activity_id FROM {0} t0"
             " LEFT JOIN {1} t1 ON t0.task_rowid = t1.rowid"
             " LEFT JOIN {2} t2 ON t0.activity_rowid = t2.rowid"
             " WHERE t1.task_id = '{3}'".format(__class__.tablename, DBObj_Task.tablename, DBObj_Activity.tablename, task_id)
            )).fetchall()
    
    @staticmethod
    def delete_by_task(dbh:DatabaseHandler, task_id:str=""):
        """Delete all relations associated with ``task_id``."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DELETE FROM {0} WHERE task_rowid IN (SELECT rowid FROM {1} WHERE task_id = '{2}')".format(__class__.tablename, DBObj_Task.tablename, task_id))
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def print_all(dbh:DatabaseHandler, task_id:str=""):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(task_id) > 0:
            sqlquery = ("SELECT t0.* FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t0.task_rowid = t1.rowid"
                        " WHERE t1.task_id = '{2}'"
                        .format(__class__.tablename, DBObj_Task.tablename, task_id)
                        )
        else:
            sqlquery = ("SELECT * FROM {0}".format(__class__.tablename))
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()

    @staticmethod
    def print_all_fk_fetch(dbh:DatabaseHandler, task_id:str=""):
        """Print relation rows including foreign-key information."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(task_id) > 0:
            sqlquery = ("SELECT t0.rowid, t1.task_id, t2.activity_id FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t0.task_rowid = t1.rowid"
                        " LEFT JOIN {2} t2 ON t0.activity_rowid = t2.rowid"
                        " WHERE t1.task_id = '{2}'"
                        .format(__class__.tablename, DBObj_Task.tablename, DBObj_Activity.tablename, task_id)
                        )
        else:
            sqlquery = ("SELECT t0.rowid, t1.task_id, t2.activity_id FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t0.task_rowid = t1.rowid"
                        " LEFT JOIN {2} t2 ON t0.activity_rowid = t2.rowid"
                        .format(__class__.tablename, DBObj_Task.tablename, DBObj_Activity.tablename)
                        )
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename} (FK Extention):")
        print(cols)
        for row in rows:
            print(list(row))
        print()

class DBObj_ActivityActionRel:
    """Node table linking activities to their constituent actions.
    
    Each row represents one action node within an activity. Edges between nodes
    are stored separately in ActivityActionEdge table.
    
    Schema:
    - action_no_pre and action_no_post are REMOVED (deprecated)
    - Edges are now managed via DBObj_ActivityActionEdge
    - Each (activity_rowid, action_no) must be unique
    """
    tablename : str = "ActivityActionRel"
    @staticmethod
    def create_tables(dbh:DatabaseHandler):
        """Create the node table schema if it doesn't already exist.
        
        Nodes store:
        - activity_rowid: FK to Activity
        - action_rowid_act: FK to Action (the concrete action referenced)
        - action_no: unique node number within the activity
        - sync_flag: optional synchronization group marker (None or 0..∞)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = (
                "CREATE TABLE IF NOT EXISTS {0}"
                "("
                " rowid             INTEGER     PRIMARY KEY,"
                " activity_rowid    INTEGER     NOT NULL,"
                " action_rowid_act  INTEGER     NOT NULL,"
                " action_no          INTEGER     NOT NULL,"
                " sync_flag          INTEGER,"
                " FOREIGN KEY (activity_rowid)"
                " REFERENCES {1} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE CASCADE,"
                " FOREIGN KEY (action_rowid_act)"
                " REFERENCES {2} (rowid)"
                " ON UPDATE CASCADE"
                " ON DELETE RESTRICT"
                ")"
                ).format(__class__.tablename, DBObj_Activity.tablename, DBObj_Action.tablename)
        dbh.execute(sqlquery)
        
        # Optional hardening: ensure (activity_rowid, action_no) is unique
        dbh.execute(
            "CREATE UNIQUE INDEX IF NOT EXISTS uniq_aar_node "
            "ON {0}(activity_rowid, action_no)".format(__class__.tablename)
        )

    @staticmethod
    def drop_tables(dbh:DatabaseHandler):
        """Remove the table and all its data."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DROP TABLE IF EXISTS {0}".format(__class__.tablename))

    @staticmethod
    def insert_nodes(dbh: DatabaseHandler, activity_id: str, nodes: list):
        """Insert multiple action nodes for an activity.
        
        Args:
            dbh: Database handler
            activity_id: Activity identifier
            nodes: List of dictionaries, each containing:
                - action_id (str): Action identifier
                - action_no (int): Sequential action number (unique within activity)
                - sync_flag (int): Synchronization flag (0=sequential, 1=parallel)
        
        Example:
            nodes = [
                {"action_id": "ACT101", "action_no": 1, "sync_flag": 0},
                {"action_id": "ACT102", "action_no": 2, "sync_flag": 0},
                {"action_id": "ACT103", "action_no": 3, "sync_flag": 1}
            ]
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return
        if activity_id is None or nodes is None or len(nodes) == 0:
            return

        # Get activity rowid
        activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
        if activity_row is None:
            print(f"ERROR: Activity '{activity_id}' not found")
            return
        activity_rowid = activity_row[0]  # First column is rowid

        # Insert nodes
        sqlquery = (
            f"INSERT INTO {__class__.tablename} "
            "(activity_rowid, action_rowid_act, action_no, sync_flag) VALUES "
            f"({activity_rowid}, "
            f"(SELECT rowid FROM {DBObj_Action.tablename} WHERE action_id = :action_id), "
            ":action_no, :sync_flag)"
        )
        try:
            dbh.execute_many(sqlquery, nodes)
        except Exception as e:
            print(f"ERROR inserting nodes: {e}")

    @staticmethod
    def fetch_nodes(dbh: DatabaseHandler, activity_id: str) -> list:
        """Fetch all action nodes for an activity with resolved action IDs.
        
        Args:
            dbh: Database handler
            activity_id: Activity identifier
        
        Returns:
            List of tuples: (action_no, action_id, sync_flag)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None:
            return []

        sqlquery = (
            f"SELECT t0.action_no, t2.action_id, t0.sync_flag "
            f"FROM {__class__.tablename} t0 "
            f"LEFT JOIN {DBObj_Activity.tablename} t1 ON t1.rowid = t0.activity_rowid "
            f"LEFT JOIN {DBObj_Action.tablename} t2 ON t2.rowid = t0.action_rowid_act "
            f"WHERE t1.activity_id = '{activity_id}' "
            f"ORDER BY t0.action_no"
        )
        rows = dbh.execute(sqlquery).fetchall()
        return rows

    @staticmethod
    def fetch_by_activity(dbh:DatabaseHandler, activity_id:str=""):
        """Return node rows filtered by activity.
        
        Args:
            dbh: Open database handle
            activity_id: Activity identifier to filter by
        
        Returns:
            List of sqlite3.Row objects, each containing:
                - rowid (int): Relation table row ID
                - activity_rowid (int): Foreign key to Activity table
                - action_rowid_act (int): Foreign key to Action table
                - action_no (int): Sequential action number (node identifier)
                - sync_flag (int): Synchronization flag
                - activity_id (str): Activity identifier (joined)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        return dbh.execute(
            ("SELECT t0.*, t1.activity_id FROM {0} t0"
             " LEFT JOIN {1} t1 ON t1.rowid = t0.activity_rowid"
             " WHERE t1.activity_id = '{2}'"
             .format(__class__.tablename, DBObj_Activity.tablename, activity_id)
            )).fetchall()
    
    @staticmethod
    def delete_by_activity(dbh:DatabaseHandler, activity_id:str=""):
        """Delete all relation rows for ``activity_id``."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        dbh.execute("DELETE FROM {0} WHERE activity_rowid IN (SELECT rowid FROM {1} WHERE activity_id = '{2}')".format(__class__.tablename, DBObj_Activity.tablename, activity_id))
    
    @staticmethod
    def schema_tables(dbh:DatabaseHandler):
        """Print the table's SQL schema and column information for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        retcur = dbh.execute("SELECT sql FROM sqlite_schema WHERE name = '{0}'".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))

        print()
        retcur = dbh.execute("pragma table_info('{0}')".format(__class__.tablename))
        cols = [description[0] for description in retcur.description]
        print(cols)
        rows = retcur.fetchall()
        for row in rows:
            print(list(row))
        print()
    

    @staticmethod
    def print_all(dbh:DatabaseHandler, activity_id:str=""):
        """Print all rows to stdout for debugging."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(activity_id) > 0:
            sqlquery = ("SELECT t0.* FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t1.rowid = t0.activity_rowid"
                        " WHERE t1.activity_id = '{2}'"
                        .format(__class__.tablename, DBObj_Activity.tablename, activity_id)
                        )
        else:
            sqlquery = ("SELECT * FROM {0}".format(__class__.tablename))
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename}:")
        print(cols)
        for row in rows:
            print(list(row))
        print()
    
    @staticmethod
    def print_all_fk_fetch(dbh:DatabaseHandler, activity_id:str=""):
        """Print node rows including resolved action identifiers."""
        if dbh is None or not isinstance(dbh, DatabaseHandler): return None
        sqlquery = None
        if len(activity_id) > 0:
            sqlquery = ("SELECT t0.rowid, t1.activity_id, t2.action_id action_id, t0.sync_flag, t0.action_no FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t1.rowid = t0.activity_rowid"
                        " LEFT JOIN {2} t2 ON t2.rowid = t0.action_rowid_act"
                        " WHERE t1.activity_id = '{3}'"
                        .format(__class__.tablename, DBObj_Activity.tablename, DBObj_Action.tablename, activity_id)
                        )
        else:
            sqlquery = ("SELECT t0.rowid, t1.activity_id, t2.action_id action_id, t0.sync_flag, t0.action_no FROM {0} t0"
                        " LEFT JOIN {1} t1 ON t1.rowid = t0.activity_rowid"
                        " LEFT JOIN {2} t2 ON t2.rowid = t0.action_rowid_act"
                        .format(__class__.tablename, DBObj_Activity.tablename, DBObj_Action.tablename)
                        )
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description] 
        rows = retcur.fetchall()
        print(f"table {__class__.tablename} (FK Extension - Nodes):")
        print(cols)
        for row in rows:
            print(list(row))
        print()


###########################################
class DBObj_ActivityActionEdge:
    """Edge table connecting action nodes within an activity (graph structure).
    
    This table implements the edge relationships between actions in an activity,
    supporting arbitrary many-to-many connections. Each edge represents a 
    predecessor→successor relationship between two action nodes.
    
    Schema:
        - activity_rowid: FK to Activity table
        - src_action_no: Source action number (predecessor)
        - dst_action_no: Destination action number (successor)
        - UNIQUE INDEX on (activity_rowid, src_action_no, dst_action_no)
        - Integrity triggers ensure edges only connect existing nodes
    """
    tablename = "ActivityActionEdge"

    @staticmethod
    def create_tables(dbh: DatabaseHandler):
        """Create the edge table with indexes and integrity triggers."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return

        # Create edge table
        sqlquery = (
            f"CREATE TABLE IF NOT EXISTS {__class__.tablename} ("
            "activity_rowid INTEGER NOT NULL, "
            "src_action_no INTEGER NOT NULL, "
            "dst_action_no INTEGER NOT NULL, "
            f"FOREIGN KEY (activity_rowid) REFERENCES {DBObj_Activity.tablename}(rowid) ON DELETE CASCADE"
            ")"
        )
        dbh.execute(sqlquery)

        # Create unique index on (activity_rowid, src_action_no, dst_action_no)
        idx_name = f"uniq_{__class__.tablename.lower()}_edge"
        sqlquery = (
            f"CREATE UNIQUE INDEX IF NOT EXISTS {idx_name} "
            f"ON {__class__.tablename} (activity_rowid, src_action_no, dst_action_no)"
        )
        dbh.execute(sqlquery)

        # Integrity trigger: Ensure src_action_no exists in ActivityActionRel
        trigger_name = f"check_src_node_{__class__.tablename.lower()}"
        sqlquery = (
            f"CREATE TRIGGER IF NOT EXISTS {trigger_name} "
            f"BEFORE INSERT ON {__class__.tablename} "
            "FOR EACH ROW "
            "BEGIN "
            f"  SELECT CASE WHEN (SELECT COUNT(*) FROM {DBObj_ActivityActionRel.tablename} "
            "    WHERE activity_rowid = NEW.activity_rowid AND action_no = NEW.src_action_no) = 0 "
            "  THEN RAISE(ABORT, 'Source node does not exist') END; "
            "END"
        )
        dbh.execute(sqlquery)

        # Integrity trigger: Ensure dst_action_no exists in ActivityActionRel
        trigger_name = f"check_dst_node_{__class__.tablename.lower()}"
        sqlquery = (
            f"CREATE TRIGGER IF NOT EXISTS {trigger_name} "
            f"BEFORE INSERT ON {__class__.tablename} "
            "FOR EACH ROW "
            "BEGIN "
            f"  SELECT CASE WHEN (SELECT COUNT(*) FROM {DBObj_ActivityActionRel.tablename} "
            "    WHERE activity_rowid = NEW.activity_rowid AND action_no = NEW.dst_action_no) = 0 "
            "  THEN RAISE(ABORT, 'Destination node does not exist') END; "
            "END"
        )
        dbh.execute(sqlquery)

    @staticmethod
    def drop_tables(dbh: DatabaseHandler):
        """Drop the edge table and its triggers."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return

        # Drop triggers first
        dbh.execute(f"DROP TRIGGER IF EXISTS check_src_node_{__class__.tablename.lower()}")
        dbh.execute(f"DROP TRIGGER IF EXISTS check_dst_node_{__class__.tablename.lower()}")
        
        # Drop table
        dbh.execute(f"DROP TABLE IF EXISTS {__class__.tablename}")

    @staticmethod
    def schema_tables(dbh: DatabaseHandler):
        """Print schema information for the edge table."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return

        sqlquery = f"SELECT sql FROM sqlite_master WHERE type='table' AND name='{__class__.tablename}'"
        rows = dbh.execute(sqlquery).fetchall()
        print(f"Schema for table {__class__.tablename}:")
        for row in rows:
            print(row[0])
        print()

        # Print indexes
        sqlquery = f"SELECT sql FROM sqlite_master WHERE type='index' AND tbl_name='{__class__.tablename}'"
        rows = dbh.execute(sqlquery).fetchall()
        print(f"Indexes for table {__class__.tablename}:")
        for row in rows:
            if row[0]:  # Skip auto-created indexes
                print(row[0])
        print()

        # Print triggers
        sqlquery = f"SELECT sql FROM sqlite_master WHERE type='trigger' AND tbl_name='{__class__.tablename}'"
        rows = dbh.execute(sqlquery).fetchall()
        print(f"Triggers for table {__class__.tablename}:")
        for row in rows:
            print(row[0])
        print()

    @staticmethod
    def fetch_by_activity(dbh: DatabaseHandler, activity_id: str) -> list:
        """Fetch all edges for a given activity.
        
        Returns:
            List of tuples: (activity_id, src_action_no, dst_action_no)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None:
            return []

        sqlquery = (
            f"SELECT t1.activity_id, t0.src_action_no, t0.dst_action_no "
            f"FROM {__class__.tablename} t0 "
            f"LEFT JOIN {DBObj_Activity.tablename} t1 ON t1.rowid = t0.activity_rowid "
            f"WHERE t1.activity_id = '{activity_id}'"
        )
        rows = dbh.execute(sqlquery).fetchall()
        return rows

    @staticmethod
    def insert_edges(dbh: DatabaseHandler, activity_id: str, edges: list):
        """Insert multiple edges for an activity.
        
        Args:
            dbh: Database handler
            activity_id: Activity identifier
            edges: List of tuples (src_action_no, dst_action_no)
        """
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return
        if activity_id is None or edges is None or len(edges) == 0:
            return

        # Get activity rowid
        activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
        if activity_row is None:
            print(f"ERROR: Activity '{activity_id}' not found")
            return
        activity_rowid = activity_row[0]  # First column is rowid

        # Insert edges
        for src_no, dst_no in edges:
            sqlquery = (
                f"INSERT INTO {__class__.tablename} "
                "(activity_rowid, src_action_no, dst_action_no) "
                f"VALUES ({activity_rowid}, {src_no}, {dst_no})"
            )
            try:
                dbh.execute(sqlquery)
            except Exception as e:
                print(f"ERROR inserting edge ({src_no}→{dst_no}): {e}")

    @staticmethod
    def delete_by_activity(dbh: DatabaseHandler, activity_id: str):
        """Delete all edges for a given activity."""
        if dbh is None or not isinstance(dbh, DatabaseHandler) or activity_id is None:
            return

        activity_row = DBObj_Activity.fetch_one(dbh, activity_id)
        if activity_row is None:
            return
        activity_rowid = activity_row[0]  # First column is rowid

        sqlquery = f"DELETE FROM {__class__.tablename} WHERE activity_rowid = {activity_rowid}"
        dbh.execute(sqlquery)

    @staticmethod
    def print_all(dbh: DatabaseHandler):
        """Print all edges with readable formatting."""
        if dbh is None or not isinstance(dbh, DatabaseHandler):
            return

        sqlquery = (
            f"SELECT t1.activity_id, t0.src_action_no, t0.dst_action_no "
            f"FROM {__class__.tablename} t0 "
            f"LEFT JOIN {DBObj_Activity.tablename} t1 ON t1.rowid = t0.activity_rowid"
        )
        retcur = dbh.execute(sqlquery)
        cols = [description[0] for description in retcur.description]
        rows = retcur.fetchall()
        
        print(f"table {__class__.tablename} (with activity FK):")
        print(cols)
        for row in rows:
            print(list(row))
        print()


###########################################
def run_cleanup():
    """Drop every managed table to reset the database state."""
    with DatabaseHandler() as dbh:
        DBObj_ActionDataPointCoords.drop_tables(dbh)
        DBObj_ActionDataPoint.drop_tables(dbh)
        DBObj_ActivityActionEdge.drop_tables(dbh)  # Drop edges before nodes
        DBObj_ActivityActionRel.drop_tables(dbh)
        DBObj_TaskActivityRel.drop_tables(dbh)
        DBObj_Action.drop_tables(dbh)
        DBObj_Activity.drop_tables(dbh)
        DBObj_Task.drop_tables(dbh)
        DBObj_ActionType.drop_tables(dbh)
        DBObj_Robot.drop_tables(dbh)

def run_creation():
    """Create all tables and seed baseline lookup data."""
    with DatabaseHandler() as dbh:
        DBObj_Robot.create_tables(dbh)
        DBObj_ActionType.create_tables(dbh)
        DBObj_Task.create_tables(dbh)
        DBObj_Activity.create_tables(dbh)
        DBObj_Action.create_tables(dbh)
        DBObj_ActionDataPoint.create_tables(dbh)
        DBObj_ActionDataPointCoords.create_tables(dbh)
        DBObj_ActivityActionRel.create_tables(dbh)
        DBObj_ActivityActionEdge.create_tables(dbh)  # Create edges after nodes
        DBObj_TaskActivityRel.create_tables(dbh)

    with DatabaseHandler() as dbh:
        DBObj_Robot.default_insert(dbh)
        DBObj_ActionType.default_insert(dbh)

def run_print_schema():
    """Print schema information for every DB object managed here."""
    with DatabaseHandler() as dbh:
        DBObj_Robot.schema_tables(dbh)
        DBObj_ActionType.schema_tables(dbh)
        DBObj_Task.schema_tables(dbh)
        DBObj_Activity.schema_tables(dbh)
        DBObj_Action.schema_tables(dbh)
        DBObj_ActionDataPoint.schema_tables(dbh)
        DBObj_ActionDataPointCoords.schema_tables(dbh)
        DBObj_TaskActivityRel.schema_tables(dbh)
        DBObj_ActivityActionRel.schema_tables(dbh)
        DBObj_ActivityActionEdge.schema_tables(dbh)  # Include edge table schema


def _refresh_activity_time(dbh: DatabaseHandler, activity_id: str) -> None:
    """Aggregate action durations to update ``Activity.activitytime``."""
    total_time = dbh.execute(
        """
        SELECT COALESCE(SUM(COALESCE(act.duration_ms, 0)), 0)
        FROM ActivityActionRel rel
        JOIN Activity ac ON ac.rowid = rel.activity_rowid
        JOIN Action act ON act.rowid = rel.action_rowid_act
        WHERE ac.activity_id = ?
        """,
        (activity_id,),
    ).fetchone()[0]
    dbh.execute(
        "UPDATE {0} SET activitytime = ? WHERE activity_id = ?".format(DBObj_Activity.tablename),
        (int(total_time), activity_id),
    )


def _refresh_task_time(dbh: DatabaseHandler, task_id: str) -> None:
    """Aggregate child activities to refresh ``Task.tasktime``."""
    total_time = dbh.execute(
        """
        SELECT COALESCE(SUM(ac.activitytime), 0)
        FROM TaskActivityRel rel
        JOIN Task t ON t.rowid = rel.task_rowid
        JOIN Activity ac ON ac.rowid = rel.activity_rowid
        WHERE t.task_id = ?
        """,
        (task_id,),
    ).fetchone()[0]
    dbh.execute(
        "UPDATE {0} SET tasktime = ? WHERE task_id = ?".format(DBObj_Task.tablename),
        (int(total_time), task_id),
    )


def run_test_insert():
    """Insert representative demo data for manual testing."""
    # Activities
    with DatabaseHandler() as dbh:
        DBObj_Activity.insert_one(dbh, activity_id="ACV001", activitytime=0)
        DBObj_Activity.insert_one(dbh, activity_id="ACV002", activitytime=0)
        DBObj_Activity.insert_one(dbh, activity_id="ACV003", activitytime=0)

    # Tasks
    with DatabaseHandler() as dbh:
        DBObj_Task.insert_one(dbh, task_id="TSK001", tasktime=0)
        DBObj_Task.insert_one(dbh, task_id="TSK002", tasktime=0)

    # Task / Activity mapping
    with DatabaseHandler() as dbh:
        DBObj_TaskActivityRel.insert_by_task(
            dbh,
            "TSK001",
            [
                {"activity_id": "ACV001"},
                {"activity_id": "ACV002"},
            ],
        )
        DBObj_TaskActivityRel.insert_by_task(
            dbh,
            "TSK002",
            [
                {"activity_id": "ACV003"},
            ],
        )

    # Actions with metadata and normalized waypoints
    with DatabaseHandler() as dbh:
        DBObj_Action.insert_one(
            dbh,
            action_id="ACT101",
            actiontype_id="Trajectory",
            robot_id="ROB1",
            metadata={
                "format_version": "trajectory_waypoints_v1.1",
                "coordinate_system": "joint_space",
            },
        )
        DBObj_ActionDataPoint.insert_by_action_id(
            dbh,
            "ACT101",
            [
                {"seq_no": 0, "t_ms": 0, "generalized_coords": [0.0, 0.5, 0.2], "gripper_width": 0.08},
                {"seq_no": 1, "t_ms": 120, "generalized_coords": [0.1, 0.6, 0.25], "gripper_width": 0.079, "gripper_force": 25.0},
            ],
        )

        DBObj_Action.insert_one(
            dbh,
            action_id="ACT102",
            actiontype_id="Trajectory",
            robot_id="ROB1",
            metadata={"format_version": "trajectory_waypoints_v1.1"},
        )
        DBObj_ActionDataPoint.insert_by_action_id(
            dbh,
            "ACT102",
            [
                {"seq_no": 0, "t_ms": 0, "generalized_coords": [0.0, -0.1, 0.0], "gripper_width": 0.082},
                {"seq_no": 1, "t_ms": 100, "generalized_coords": [0.2, -0.05, 0.05], "gripper_width": 0.081},
                {"seq_no": 2, "t_ms": 240, "generalized_coords": [0.3, 0.0, 0.1], "gripper_width": 0.08},
            ],
        )

        DBObj_Action.insert_one(
            dbh,
            action_id="ACT201",
            actiontype_id="MoveToConfig",
            robot_id="ROB2",
            metadata={"coordinate_system": "joint_space"},
        )
        DBObj_ActionDataPoint.insert_by_action_id(
            dbh,
            "ACT201",
            [
                {"seq_no": 0, "t_ms": 0, "generalized_coords": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {"seq_no": 1, "t_ms": 200, "generalized_coords": [0.1, -0.1, 0.05, 0.0, 0.0, 0.0]},
            ],
        )

        DBObj_Action.insert_one(
            dbh,
            action_id="ACT301",
            actiontype_id="Gripper",
            robot_id="ROB3",
            metadata={"coordinate_system": "tool"},
        )
        DBObj_ActionDataPoint.insert_by_action_id(
            dbh,
            "ACT301",
            [
                {"seq_no": 0, "t_ms": 0, "generalized_coords": [], "gripper_width": 0.085, "gripper_force": 10.0},
                {"seq_no": 1, "t_ms": 150, "generalized_coords": [], "gripper_width": 0.02, "gripper_force": 40.0},
            ],
        )

    # Activity sequencing using new node+edge pattern
    with DatabaseHandler() as dbh:
        # ACV001: Sequential chain ACT101 → ACT102
        DBObj_ActivityActionRel.insert_nodes(
            dbh,
            "ACV001",
            [
                {"action_id": "ACT101", "action_no": 1, "sync_flag": 0},
                {"action_id": "ACT102", "action_no": 2, "sync_flag": 0},
            ],
        )
        DBObj_ActivityActionEdge.insert_edges(
            dbh,
            "ACV001",
            [(1, 2)],  # ACT101 → ACT102
        )
        _refresh_activity_time(dbh, "ACV001")

        # ACV002: Single action, no edges
        DBObj_ActivityActionRel.insert_nodes(
            dbh,
            "ACV002",
            [
                {"action_id": "ACT201", "action_no": 1, "sync_flag": 0},
            ],
        )
        _refresh_activity_time(dbh, "ACV002")

        # ACV003: Single action, no edges
        DBObj_ActivityActionRel.insert_nodes(
            dbh,
            "ACV003",
            [
                {"action_id": "ACT301", "action_no": 1, "sync_flag": 0},
            ],
        )
        _refresh_activity_time(dbh, "ACV003")
        _refresh_task_time(dbh, "TSK001")
        _refresh_task_time(dbh, "TSK002")


def run_test_insert2():
    """Load an alternative demo payload using new node+edge pattern."""
    with DatabaseHandler() as dbh:
        DBObj_Activity.insert_one(dbh, activity_id="ACV1", activitytime=0)

        # Create actions
        for index in range(1, 4):
            DBObj_Action.insert_one(
                dbh,
                action_id=f"Action{index}",
                actiontype_id="Trajectory",
                robot_id="ROB1",
                metadata={"format_version": "trajectory_waypoints_v1.1"},
            )
            DBObj_ActionDataPoint.insert_by_action_id(
                dbh,
                f"Action{index}",
                [
                    {"seq_no": 0, "t_ms": 0, "generalized_coords": [0.05 * index, 0.02 * index]},
                    {"seq_no": 1, "t_ms": 150, "generalized_coords": [0.1 * index, 0.04 * index]},
                ],
            )

        # Insert nodes: Action1 (sequential), Action2 (sequential), Action3 (parallel)
        DBObj_ActivityActionRel.insert_nodes(
            dbh,
            "ACV1",
            [
                {"action_id": "Action1", "action_no": 1, "sync_flag": 0},
                {"action_id": "Action2", "action_no": 2, "sync_flag": 0},
                {"action_id": "Action3", "action_no": 3, "sync_flag": 1},
            ],
        )

        # Insert edges: Action1 → Action2 → Action3
        DBObj_ActivityActionEdge.insert_edges(
            dbh,
            "ACV1",
            [
                (1, 2),  # Action1 → Action2
                (2, 3),  # Action2 → Action3
            ],
        )
        _refresh_activity_time(dbh, "ACV1")

    # Verify data
    with DatabaseHandler() as dbh:
        print()
        print("=== Node+Edge Structure for ACV1 ===")
        print("Nodes:")
        nodes = DBObj_ActivityActionRel.fetch_nodes(dbh, "ACV1")
        for node in nodes:
            print(f"  action_no={node[0]}, action_id={node[1]}, sync_flag={node[2]}")
        
        print("\nEdges:")
        edges = DBObj_ActivityActionEdge.fetch_by_activity(dbh, "ACV1")
        for edge in edges:
            print(f"  {edge[1]} → {edge[2]}")
        print()
        

###########################################
if __name__ == '__main__':
    run_cleanup()
    run_creation()
    run_test_insert()
    run_test_insert2()
    run_print_schema()

    with DatabaseHandler() as dbh:
        DBObj_Robot.print_all(dbh)
        DBObj_ActionType.print_all(dbh)
        DBObj_Action.print_all(dbh)
        DBObj_Activity.print_all(dbh)
        DBObj_Task.print_all(dbh)
        DBObj_TaskActivityRel.print_all_fk_fetch(dbh)
        DBObj_ActivityActionRel.print_all_fk_fetch(dbh)
        DBObj_ActivityActionEdge.print_all(dbh)

    print("Demo complete. Use run_cleanup() to reset the database if needed.")
```
