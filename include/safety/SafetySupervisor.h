#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

class SafetySupervisor {
public:
    virtual ~SafetySupervisor() = default;
    virtual bool checkSafety();
};

#endif