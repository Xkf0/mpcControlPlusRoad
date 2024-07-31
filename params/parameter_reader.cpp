#include "parameter_reader.h"
#include <map>
#include <mutex>
#include <shared_mutex>

namespace rac
{
    typedef std::map<std::string, std::string> ParameterMap;
    ParameterMap g_params;
    std::mutex cell_mutex; // 单元锁
    namespace param
    {
        bool Has(const std::string &key)
        {
            std::lock_guard<std::mutex> locker(cell_mutex);
            return g_params.find(key) != g_params.end();
        }

        void Set(const std::string &key, const std::string &value)
        {
            std::lock_guard<std::mutex> locker(cell_mutex);
            g_params[key] = value;
        }

        bool Get(const std::string &key, std::string &value)
        {
            std::lock_guard<std::mutex> locker(cell_mutex);
            auto finder = g_params.find(key);
            if (finder != g_params.end())
            {
                value = finder->second;
                return true;
            }
            return false;
        }

        void Load(const std::string &param_fname, bool clear)
        {
            if (clear)
            {
                g_params.clear();
            }
            std::ifstream fin(param_fname.c_str());
            if (!fin)
            {
                std::cerr << "Open file error:" << param_fname << std::endl;
                return;
            }
            while (!fin.eof())
            {
                std::string str;
                getline(fin, str);
                if (str[0] == '#' || str[0] == '//')
                    continue; // 以‘＃’‘//'开头的是注释
                int pos = str.find("=");
                if (pos == -1)
                {
                    continue;
                }
                std::string key = str.substr(0, pos);
                std::string value = str.substr(pos + 1, str.length());
                g_params[key] = value;
                if (!fin.good())
                    break;
            }
            fin.close();
        }
    }

    ParameterReader::ParameterReader() {}

    ParameterReader::ParameterReader(const std::string &param_fname)
    {
        param::Load(param_fname, true);
    }

    ParameterReader::~ParameterReader() {}

    bool ParameterReader::hasParam(const std::string &key) const
    {
        return param::Has(key);
    }

} // namespace robos