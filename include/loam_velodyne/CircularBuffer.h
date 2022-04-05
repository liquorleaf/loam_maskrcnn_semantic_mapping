#ifndef LOAM_CIRCULARBUFFER_H
#define LOAM_CIRCULARBUFFER_H

#include <cstdlib>
#include <vector>

// 1个工具类：CircularBuffer

namespace loam {


/** \brief Simple circular buffer implementation for storing data history.
 * 数组型循环队列；成员数据的类型用了模板
 * @tparam T The buffer element type.
 */
template <class T>
class CircularBuffer {
public:
  // 构造函数：默认容量200
  CircularBuffer(const size_t& capacity = 200)
        : _capacity(capacity),
          _size(0),
          _startIdx(0)
  {
    _buffer = new T[capacity];
  };

  // 析构函数
  ~CircularBuffer()
  {
    delete[] _buffer;
    _buffer = NULL;
  }

  // 提取buffer现在存储元素数
  /** \brief Retrieve the buffer size.
   *
   * @return the buffer size
   */
  const size_t& size() {
    return _size;
  }

  // 提取buffer现在最大容量
  /** \brief Retrieve the buffer capacity.
   *
   * @return the buffer capacity
   */
  const size_t& capacity() {
    return _capacity;
  }

  // 使buffer至少具有某最大容量（不足时创建新的更大的并把已有有效元素复制进去）（重置“最早”标记索引到[0]）
  // 注意：若之前已经在满的状态下入队，发生覆盖，_startIdx改变，则reqCapacity重置_startIdx到[0]会使覆盖前后的时间顺序丢失
  /** \brief Ensure that this buffer has at least the required capacity.
   *
   * @param reqCapacity the minimum required capacity
   */
  void ensureCapacity(const int& reqCapacity) {
    if (reqCapacity > 0 && _capacity < reqCapacity) {
      // create new buffer and copy (valid) entries
      T* newBuffer = new T[reqCapacity];
      for (size_t i = 0; i < _size; i++) {
        newBuffer[i] = (*this)[i];
      }

      // switch buffer pointers and delete old buffer
      T* oldBuffer = _buffer;
      _buffer = newBuffer;
      _startIdx = 0;

      delete[] oldBuffer;
    }
  }

  // buffer是否为空
  /** \brief Check if the buffer is empty.
   *
   * @return true if the buffer is empty, false otherwise
   */
  bool empty() {
    return _size == 0;
  }

  //重载[]，提取buffer中存储的有效元素的第i个
  /** \brief Retrieve the i-th element of the buffer.
   *
   *
   * @param i the buffer index
   * @return the element at the i-th position
   */
  const T& operator[](const size_t& i) {
    return _buffer[(_startIdx + i) % _capacity];
  }

  //提取buffer中存储的有效元素的第1个（最先进入的）（为空时提取[0]处的）
  /** \brief Retrieve the first (oldest) element of the buffer.
   *
   * @return the first element
   */
  const T& first() {
    return _buffer[_startIdx];
  }

  //提取buffer中存储的有效元素的最后一个（最后进入的）（为空时提取[0]处的）
  /** \brief Retrieve the last (latest) element of the buffer.
   *
   * @return the last element
   */
  const T& last() {
    size_t idx = _size == 0 ? 0 : (_startIdx + _size - 1) % _capacity;
    return _buffer[idx];
  }

  // 新元素入队：从尾部；若已满，覆盖最早的，“最早”索引标记往后推
  // 注意：因为只有入队函数，没有主动出队函数，所以startIdx只有在超容量上限导致发生覆盖时，才会改变
  /** \brief Push a new element to the buffer.
   *
   * If the buffer reached its capacity, the oldest element is overwritten.
   *
   * @param element the element to push
   */
  void push(const T& element) {
    if (_size < _capacity) {
      _buffer[_size] = element;
      _size++;
    } else {
      _buffer[_startIdx] = element;
      _startIdx = (_startIdx + 1) % _capacity;
    }
  }

// 成员：注意总容量_capacity，现在存储元素个数_size，现在存储元素中最早的位于_startIdx处
private:
  size_t _capacity;   ///< buffer capacity
  size_t _size;       ///< current buffer size
  size_t _startIdx;   ///< current start index
  T* _buffer;         ///< internal element buffer
};

} // end namespace loam

#endif //LOAM_CIRCULARBUFFER_H
