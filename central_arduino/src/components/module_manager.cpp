#include "module_manager.h"

ModuleManager::ModuleManager()
{
}

Status ModuleManager::loop()
{
    for (BaseModule *module : this->modules)
    {
        Status s = module->loop();
        if (s == Status::FAILED)
        {
 //           Serial.print("Failed: ");
 //           Serial.println(module->name);
        }
    }
    return Status::OK;
}

Status ModuleManager::setupModule(BaseModule *module)
{
    module->setup();
    modules.push_back(module);
    return Status::OK;
}