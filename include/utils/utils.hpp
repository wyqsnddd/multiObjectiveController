# ifndef UTILS_HPP
# define UTILS_HPP

# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

# ifndef pt_namespace
# define pt_namespace
namespace pt = boost::property_tree;
# endif


	template <typename T>
std::vector<T> as_vector(pt::ptree const& ptIn, pt::ptree::key_type const& key)
{
	std::vector<T> r;
	for (auto& item : ptIn.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}


# endif
