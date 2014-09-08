#pragma once

template<class T, int max_elements>
class CircularQueue
{
public:
	CircularQueue():
	start(0),
	count(0)
	{

	}
	~CircularQueue()
	{

	}

	// push a element to end of the queue
	// return 0 if success
	//        -1 on error (queue full)
	int push(const T &v)
	{
		if (count >= max_elements)
			return -1;

		elements[(start+count)%max_elements] = v;
		count++;
	}

	// pop and return the first element out of the queue.
	// return 0 if success and *out modified if valid pointer passed.
	//        -1 if no element available and *out remain untouched.
	int pop(T *out)
	{
		if (count == 0)
			return -1;

		if (out)
			*out = elements[start];
		start = (start+1)%max_elements;
		count--;

		return 0;
	}


	// peak the elements at the specified index of queues.
	// return 0 if success and *out if modified.
	//        -1 if element not found and *out remain untouched or invaild out pointer.
	int peak(int index, T*out)
	{
		if (index >= count)
			return -1;

		if (!out)
			return -1;

		*out = elements[(start+index)%max_elements];
		return 0;
	}


	// pop and discards n elements at the top of the queue.
	// return number of elements removed.
	int pop_n(int _count)
	{
		if (_count>count)
			_count = count;

		start = (start + _count)%max_elements;
		count -= _count;

		return _count;			
	}

protected:
	T elements[max_elements];
	int start;
	int count;
};