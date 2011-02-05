#include "RSTApp.h"
#include "TemplateTab.h"

extern wxNotebook* tabView;

class TemplateTabApp : public RSTApp {
	virtual void AddTabs() {
		tabView->AddPage(new TemplateTab(tabView), wxT("TemplateTab"));
	}
};

IMPLEMENT_APP(TemplateTabApp)