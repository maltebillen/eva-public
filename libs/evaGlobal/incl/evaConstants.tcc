template<class T>
inline std::vector<T> eva::Helper::ConcatVectors(const std::vector<T>& a, const std::vector<T>& b)
{
	std::vector<T> res = a;
	res.insert(res.end(), b.begin(), b.end());
	return res;
}
