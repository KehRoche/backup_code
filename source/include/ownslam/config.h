//#ifdef CONFIG_H
#define CONFIG_H

#include "ownslam/common_include.h"

namespace ownslam
{
    class Config
    {
        private:
            static std::shared_ptr<Config> config_;
            cv::FileStorage file_;

            Config(){}
        public:
            ~Config();
            
            static void setParamterFile(const std::string& filename);
            //template function datetype was decided by input
            template< typename T>
            static T get(const std::string& key)
            {
                return T(Config::config_->file_[key]);
            }
    };

} // namespace ownslam
#endif