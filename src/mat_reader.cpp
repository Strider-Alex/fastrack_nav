#include <fastrack_nav/mat_reader.h>

namespace fastrack_local_planner {

// Open a file. Return bool upon success.
bool MATReader::Open(const std::string& file_name) {
  // Close any open file before opening a new file.
  if (IsOpen()) Close();

  // Open the new file.
  mat_fp_ = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  return IsOpen();
}

// Read scalar. Returns bool indicating success.
bool MATReader::ReadScalar(const std::string& field_name,
                                  double* value) {
  if (!IsOpen()) return false;

  // Make sure 'value' is non-null.
  if (!value) return false;

  // Extract this variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_DOUBLE) return false;

  *value = *static_cast<double*>(variable->data);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read string. Returns bool indicating success.
bool MATReader::ReadString(const std::string& field_name,
                                  std::string* value) {
  if (!IsOpen()) return false;

  // Make sure 'value' is non-null.
  if (!value) return false;

  // Extract this variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable) return false; // || variable->data_type != MAT_T_UTF8) return false;

  value->assign(static_cast<char*>(variable->data));

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read scalar. Returns bool indicating success.
bool MATReader::ReadScalar(const std::string& field_name,
                                  size_t* value) {
  if (!IsOpen()) return false;

  // Make sure 'value' is non-null.
  if (!value) return false;

  // Extract this variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_UINT64) return false;

  *value = *static_cast<size_t*>(variable->data);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read vector. Returns bool indicating success.
bool MATReader::ReadVector(const std::string& field_name,
                                  std::vector<double>* values) {
  if (!IsOpen()) return false;

  // Make sure 'values' is non-null.
  if (!values) return false;
  values->clear();

  // Extract variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_DOUBLE) return false;

  size_t num_elements = variable->nbytes / variable->data_size;
  for (size_t ii = 0; ii < num_elements; ii++)
    values->emplace_back(static_cast<double*>(variable->data)[ii]);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read vector. Returns bool indicating success.
bool MATReader::ReadVector(const std::string& field_name,
                                  std::vector<size_t>* values) {
  if (!IsOpen()) return false;

  // Make sure 'values' is non-null.
  if (!values) return false;
  values->clear();

  // Extract variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_UINT64) return false;

  size_t num_elements = variable->nbytes / variable->data_size;
  for (size_t ii = 0; ii < num_elements; ii++)
    values->emplace_back(static_cast<size_t*>(variable->data)[ii]);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

} // namespace fastrack