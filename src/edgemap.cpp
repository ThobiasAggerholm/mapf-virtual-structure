#include "../inc/edgemap.hpp"
#include <cassert>
#include <fstream>

EdgeMap::EdgeMap()
{

}

EdgeMap::EdgeMap(EdgeMap &src)
 : m_edge_map{std::move(src.m_edge_map)}
{
    for(auto cit = m_edge_map.cbegin(); cit != m_edge_map.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            m_mutex_map[cit->first][cit_inner->first];
        }
    }
}

EdgeMap::EdgeMap(const EdgeMap &src)
 : m_edge_map{src.m_edge_map}
{
    for(auto cit = m_edge_map.cbegin(); cit != m_edge_map.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            m_mutex_map[cit->first][cit_inner->first];
        }
    }
}

EdgeMap::~EdgeMap()
{

}

EdgeMap& EdgeMap::operator=(EdgeMap &rhs)
{
    m_edge_map = std::move(rhs.m_edge_map);
    for(auto cit = m_edge_map.cbegin(); cit != m_edge_map.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            m_mutex_map[cit->first][cit_inner->first];
        }
    }
    return *this;
}

EdgeMap& EdgeMap::operator=(const EdgeMap &rhs)
{
    m_edge_map = rhs.m_edge_map;
    for(auto cit = m_edge_map.cbegin(); cit != m_edge_map.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            m_mutex_map[cit->first][cit_inner->first];
        }
    }

    return *this;
}

EdgeMap EdgeMap::operator+(EdgeMap const & rhs) const
{
    EdgeMap result{*this};
    assert(result.size() == rhs.size());
    for(auto cit = rhs.cbegin(); cit != rhs.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            result.add(cit->first, cit_inner->first, cit_inner->second);
        }
    }

    return result;
}

EdgeMap EdgeMap::operator/(double const & rhs) const
{
    EdgeMap result{*this};
    for(auto cit = result.cbegin(); cit != result.cend(); ++cit)
    {
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            result.write(cit->first, cit_inner->first, cit_inner->second / rhs);
        }
    }

    return result;
}



double EdgeMap::read(Node const* src, Node const* dst)
{
    double val;
    m_mutex_map.at(src).at(dst).lock();
    val = m_edge_map.at(src).at(dst);
    m_mutex_map.at(src).at(dst).unlock();

    return val;
}

double EdgeMap::read(Node const* src, Node const* dst) const
{
    return m_edge_map.at(src).at(dst);
}

void  EdgeMap::write(Node const* src, Node const* dst, double val)
{
    if(m_edge_map.find(src) == m_edge_map.end())
    {
        m_edge_map[src][dst];
        m_mutex_map[src][dst];
    }
    else if(m_edge_map[src].find(dst) == m_edge_map[src].end())
    {
        m_edge_map[src][dst];
        m_mutex_map[src][dst];
    }

    m_mutex_map.at(src).at(dst).lock();
    m_edge_map[src][dst] = val;
    m_mutex_map.at(src).at(dst).unlock();

    return;
}
void  EdgeMap::add(Node const* src, Node const* dst, double val)
{
    if(m_edge_map.find(src) == m_edge_map.end())
    {
        m_edge_map[src][dst] = 0;
        m_mutex_map[src][dst];
    }
    else if(m_edge_map[src].find(dst) == m_edge_map[src].end())
    {
        m_edge_map[src][dst] = 0;
        m_mutex_map[src][dst];
    }

    m_mutex_map.at(src).at(dst).lock();
    m_edge_map[src][dst] += val;
    m_mutex_map.at(src).at(dst).unlock();

    return;
}

double EdgeMap::reduce()
{
    return std::reduce(EXECUTION_POLICY_EDGEMAP, m_edge_map.cbegin(), m_edge_map.cend(), 0.0,
    [this](double value, const std::unordered_map<Node const*, std::unordered_map<Node const*, double>>::value_type& p)
    {
        Node const* src = p.first;
        return std::reduce(EXECUTION_POLICY_EDGEMAP, p.second.cbegin(), p.second.cend(), value,
        [this, src](double value, const std::unordered_map<Node const*, double>::value_type& p_i)
        {
            
            m_mutex_map.at(src).at(p_i.first).lock();
            double new_val = value + p_i.second;
            m_mutex_map.at(src).at(p_i.first).unlock();
            return new_val;
        });
        
    });
}

size_t EdgeMap::size() const
{
    return std::reduce(EXECUTION_POLICY_EDGEMAP, m_edge_map.cbegin(), m_edge_map.cend(), 0,
    [this](int value, const std::unordered_map<Node const*, std::unordered_map<Node const*, double>>::value_type& p)
    {
        return value + p.second.size();   
    });
}

void EdgeMap::export_geometric(std::string fname, int map_width, int map_height) const
{
    std::vector<std::vector<std::vector<double>>> highways(map_width, std::vector<std::vector<double>>(map_height, std::vector<double>(2, 0.0)));

    for(auto cit = m_edge_map.cbegin(); cit != m_edge_map.cend(); ++cit)
    {
        Node const* src = cit->first;
        int x = src->get_id() % map_width;
        int y = src->get_id() / map_width;
        for(auto cit_inner = cit->second.cbegin(); cit_inner != cit->second.cend(); ++cit_inner)
        {
            Node const* dst = cit_inner->first;
            int x_dst = dst->get_id() % map_width;
            int y_dst = dst->get_id() / map_width;
            if(x == x_dst) //South bound
            {
                if(y < y_dst)
                {
                    highways[x][y][0] = cit_inner->second - 0.5;
                }
            }
            else if(y == y_dst)
            {
                if(x < x_dst)
                {
                    highways[x][y][1] = cit_inner->second - 0.5;
                }
            }
        }
    }

    std::ofstream file(fname);
    for(int i = 0; i < map_width; ++i)
    {
        for(int j = 0; j < map_height; ++j)
        {
            for (size_t k = 0; k < 2; k++)
            {
                file << i << "," << j << "," << k << "," << highways[i][j][k] << std::endl;
            }
        }
    }
}
