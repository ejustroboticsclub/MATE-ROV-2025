// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rov_debug_interfaces:msg/Imu.idl
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
#include "rov_debug_interfaces/msg/detail/imu__struct.h"
#include "rov_debug_interfaces/msg/detail/imu__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rov_debug_interfaces__msg__imu__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[34];
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
    assert(strncmp("rov_debug_interfaces.msg._imu.Imu", full_classname_dest, 33) == 0);
  }
  rov_debug_interfaces__msg__Imu * ros_message = _ros_message;
  {  // acc_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acc_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // acc_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acc_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // acc_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acc_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rov_debug_interfaces__msg__imu__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Imu */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rov_debug_interfaces.msg._imu");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Imu");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rov_debug_interfaces__msg__Imu * ros_message = (rov_debug_interfaces__msg__Imu *)raw_ros_message;
  {  // acc_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acc_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acc_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acc_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
