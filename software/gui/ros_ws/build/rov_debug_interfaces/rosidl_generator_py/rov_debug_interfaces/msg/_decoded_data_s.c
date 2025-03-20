// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rov_debug_interfaces/msg/detail/decoded_data__struct.h"
#include "rov_debug_interfaces/msg/detail/decoded_data__functions.h"

bool rov_debug_interfaces__msg__imu__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rov_debug_interfaces__msg__imu__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rov_debug_interfaces__msg__decoded_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rov_debug_interfaces.msg._decoded_data.DecodedData", full_classname_dest, 50) == 0);
  }
  rov_debug_interfaces__msg__DecodedData * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu");
    if (!field) {
      return false;
    }
    if (!rov_debug_interfaces__msg__imu__convert_from_py(field, &ros_message->imu)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // thruster_current_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_current_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_current_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_current_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_current_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_current_6
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_current_6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_current_6 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thruster_pwm_6
    PyObject * field = PyObject_GetAttrString(_pymsg, "thruster_pwm_6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thruster_pwm_6 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // indicator_6
    PyObject * field = PyObject_GetAttrString(_pymsg, "indicator_6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->indicator_6 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heartbeat_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_1");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->heartbeat_1 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heartbeat_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_2");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->heartbeat_2 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heartbeat_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_3");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->heartbeat_3 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heartbeat_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_4");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->heartbeat_4 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // connection_percentage_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_percentage_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->connection_percentage_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // connection_percentage_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_percentage_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->connection_percentage_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // connection_percentage_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_percentage_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->connection_percentage_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // connection_percentage_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_percentage_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->connection_percentage_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // arm_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "arm_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->arm_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // arm_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "arm_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->arm_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // arm_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "arm_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->arm_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // arm_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "arm_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->arm_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rov_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "rov_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rov_depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rov_debug_interfaces__msg__decoded_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DecodedData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rov_debug_interfaces.msg._decoded_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DecodedData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rov_debug_interfaces__msg__DecodedData * ros_message = (rov_debug_interfaces__msg__DecodedData *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu
    PyObject * field = NULL;
    field = rov_debug_interfaces__msg__imu__convert_to_py(&ros_message->imu);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_current_6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_current_6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_current_6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thruster_pwm_6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thruster_pwm_6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thruster_pwm_6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // indicator_6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->indicator_6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "indicator_6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_1
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->heartbeat_1 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_2
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->heartbeat_2 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_3
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->heartbeat_3 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_4
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->heartbeat_4 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_percentage_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->connection_percentage_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_percentage_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_percentage_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->connection_percentage_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_percentage_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_percentage_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->connection_percentage_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_percentage_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_percentage_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->connection_percentage_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_percentage_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arm_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->arm_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arm_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arm_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->arm_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arm_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arm_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->arm_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arm_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arm_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->arm_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arm_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rov_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rov_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rov_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
