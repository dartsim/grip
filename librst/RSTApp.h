#ifndef RSTAPP_H_
#define RSTAPP_H_


class RSTApp : public wxApp
{
public:
    virtual bool OnInit();
    void AddTab();
};

DECLARE_APP(RSTApp)

#endif /* RSTAPP_H_ */
