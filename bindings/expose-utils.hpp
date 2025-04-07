#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <vector>

using namespace boost::python;

// From http://bazaar.launchpad.net/~yade-dev/yade/trunk/view/head:/py/wrapper/customConverters.cpp#L127
// Allow conversions from list to some std::vector
template <typename containedType>
struct custom_vector_from_seq
{
  custom_vector_from_seq()
  {
    converter::registry::push_back(&convertible, &construct, type_id<std::vector<containedType>>());
  }
  static void* convertible(PyObject* obj_ptr)
  {
    // the second condition is important, for some reason otherwise there were
    // attempted conversions of Body to list which failed afterwards.
    if (!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__"))
      return 0;
    return obj_ptr;
  }
  static void construct(PyObject* obj_ptr, converter::rvalue_from_python_stage1_data* data)
  {
    void* storage = ((converter::rvalue_from_python_storage<std::vector<containedType>>*)(data))->storage.bytes;
    new (storage) std::vector<containedType>();
    std::vector<containedType>* v = (std::vector<containedType>*)(storage);
    int l = PySequence_Size(obj_ptr);
    if (l < 0)
      abort(); /*std::cerr<<"l="<<l<<";
                  "<<typeid(containedType).name()<<std::endl;*/
    v->reserve(l);
    for (int i = 0; i < l; i++)
    {
      v->push_back(extract<containedType>(PySequence_GetItem(obj_ptr, i)));
    }
    data->convertible = storage;
  }
};

template <typename T>
bool is_not_registered()
{
  boost::python::type_info info = boost::python::type_id<T>();
  const boost::python::converter::registration* reg = boost::python::converter::registry::query(info);

  return reg == NULL || (*reg).m_to_python == NULL;
}

/**
 * @brief Exposes a given type for std::vector
 * @tparam T type
 * @param name name of the type as exposed in Python
 */
template <typename T>
void exposeStdVector(const std::string& class_name)
{
  typedef typename std::vector<T> vector_T;

  if (is_not_registered<vector_T>()) {
    class_<vector_T>(class_name.c_str()).def(vector_indexing_suite<vector_T>());

    custom_vector_from_seq<T>();
  }
}

template <typename K, typename V>
void exposeStdMap(const std::string& class_name)
{
  typedef typename std::map<K, V> map_K_V;

  class_<map_K_V>(class_name.c_str()).def(map_indexing_suite<map_K_V>());
}

template <typename K, typename V>
void update_map(std::map<K, V>& map_, boost::python::dict& py_dict)
{
  boost::python::list keys = py_dict.keys();
  for (int i = 0; i < len(keys); ++i)
  {
    boost::python::extract<K> extracted_key(keys[i]);
    if (!extracted_key.check())
    {
      std::cout << "Key invalid, map might be incomplete" << std::endl;
      continue;
    }
    K key = extracted_key;

    boost::python::extract<V> extracted_val(py_dict[key]);
    if (!extracted_val.check())
    {
      std::cout << "Value invalid, map might be incomplete" << std::endl;
      continue;
    }
    V value = extracted_val;

    map_[key] = value;
  }
}