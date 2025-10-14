#include "fatfsff.hpp"
#include "mklog.hpp"
#include <string>
#include <map>
#include <vector>

class mConfig
{
public:
    struct ConfigItem {
        std::string key;
        std::string value;
        
        ConfigItem() {}
        ConfigItem(const std::string& k, const std::string& v)
            : key(k), value(v) {}
    };

    static mConfig* getInstance()
    {
        static mConfig configx;
        return &configx;
    }
    
    // 设置配置文件名
    void setFileName(const std::string& fname) 
    {
        if (bFileOpen) {
            configFile.close();
            bFileOpen = false;
        }
        fileName = fname;
    }

    std::string getFileName() const {return fileName;}
    
    // 加载配置文件
    bool load()
    {
        FRESULT res;
        if((res = configFile.open(fileName.c_str(), FA_READ | FA_OPEN_EXISTING)) == FRESULT::FR_OK)
        {
            ALOGI("open config file %s successful\r\n", fileName.c_str());
            bFileOpen = true;
            
            while(!configFile.eof())
            {
                std::string line(256, '\0');
                configFile.gets(line.data(), line.size());
                line.erase(line.find_last_not_of("\r\n") + 1);
                
                // 解析键值对
                std::string key, value;
                size_t pos = line.find('=');
                if (pos != std::string::npos) {
                    key = line.substr(0, pos);
                    value = line.substr(pos + 1);
                } else {
                    continue;
                }
                
                // 移除首尾空格
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                configItems[key] = ConfigItem(key, value);
            }
            configFile.close();
            bFileOpen = false;
        }
        else
        {
            ALOGE("open config file %s failed, Reason: (%s)\r\n", fileName.c_str(), mFatFs::errToStr(res));
            bFileOpen = false;
            return false;
        }
        return true;
    }

    // 保存配置文件
    bool save(std::vector<ConfigItem>& content)
    {
        if (bFileOpen) {
            configFile.close();
            bFileOpen = false;
        }
        load();

        for(auto& item : content)
        {
            configItems[item.key] = item;
        }
        FRESULT res;
        if((res = configFile.open(fileName.c_str(), FA_CREATE_ALWAYS | FA_WRITE)) == FRESULT::FR_OK)
        {
            ALOGI("create config file %s successful\r\n", fileName.c_str());
            bFileOpen = true;
            
            for(auto& item : configItems)
            {
                unsigned int bw;
                std::string contentStr = item.first + "=" + item.second.value + "\r\n";
                configFile.write(contentStr.c_str(), contentStr.size(), &bw);
            }
            configFile.sync();
            configFile.close();
            bFileOpen = false;
            return true;
        }
        else
        {
            ALOGE("create config file %s failed, Reason: (%s)\r\n", fileName.c_str(), mFatFs::errToStr(res));
            bFileOpen = false;
        }
        return false;
    }
    bool ready()
    {
        return exists();
    }
    std::string getValue(const std::string& key)
    {
        if(configItems.find(key) != configItems.end())
        {
            return configItems[key].value;
        }
        return "";
    }
    // 检查配置文件是否存在
    bool exists()
    {
        FILINFO fno;
        return f_stat(fileName.c_str(), &fno) == FR_OK;
    }
    
    // 备份配置文件
    bool backup(const std::string& backupName)
    {
        if (!exists()) return false;
        
        std::string backupPath = fileName + ".bak";
        if (!backupName.empty()) {
            backupPath = backupName;
        }
        
        // 复制文件
        return copyFile(fileName, backupPath);
    }
    
    // 恢复配置文件
    bool restore(const std::string& backupName)
    {
        std::string backupPath = fileName + ".bak";
        if (!backupName.empty()) {
            backupPath = backupName;
        }
        
        return copyFile(backupPath, fileName);
    }
    
private:
    mConfig(const std::string& fileName = "0:/config.ini"):bFileOpen(false),fileName(fileName)
    {

    }
    
    virtual ~mConfig() 
    {
        if (bFileOpen) {
            configFile.close();
        }
    }
    
    bool readFileContent(std::string& content)
    {
        content.clear();
        char buffer[256];
        unsigned int bytesRead;
        unsigned int totalRead = 0;
        
        while (true) {
            FRESULT res = configFile.read((uint8_t*)buffer, sizeof(buffer)-1, &bytesRead);
            if (res != FR_OK || bytesRead == 0) {
                break;
            }
            
            buffer[bytesRead] = '\0';
            content += buffer;
            totalRead += bytesRead;
            
            if (bytesRead < sizeof(buffer)-1) {
                break;  // 文件结束
            }
        }
        
        printf("读取配置文件完成，共 %d 字节\n", totalRead);
        return totalRead > 0;
    }
    
    bool writeFileContent(const std::string& content)
    {
        unsigned int bw;
        FRESULT res = configFile.write((const uint8_t*)content.c_str(), content.length(), &bw);
        
        if (res == FR_OK && bw == content.length()) {
            configFile.sync();
            printf("配置文件写入成功，共 %d 字节\n", bw);
            return true;
        } else {
            printf("配置文件写入失败\n");
            return false;
        }
    }
    
    bool copyFile(const std::string& src, const std::string& dst)
    {
        // 简单的文件复制实现
        mFile srcFile, dstFile;
        
        if (srcFile.open(src.c_str(), FA_READ) != FR_OK) return false;
        if (dstFile.open(dst.c_str(), FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
            srcFile.close();
            return false;
        }
        
        char buffer[512];
        unsigned int br, bw;
        bool success = true;
        
        while (success) {
            if (srcFile.read((uint8_t*)buffer, sizeof(buffer), &br) != FR_OK || br == 0) {
                break;
            }
            
            if (dstFile.write((uint8_t*)buffer, br, &bw) != FR_OK || bw != br) {
                success = false;
                break;
            }
        }
        
        srcFile.close();
        dstFile.close();
        return success;
    }
    
private:
    bool bFileOpen;
    mFile configFile;
    std::string fileName;
    std::map<std::string, ConfigItem> configItems;
};