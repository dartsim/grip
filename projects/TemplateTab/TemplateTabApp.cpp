#include "GRIPApp.h"
#include "TemplateTab.h"

extern wxNotebook* tabView;

class TemplateTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new TemplateTab(tabView), wxT("TemplateTab"));
	}
};

IMPLEMENT_APP(TemplateTabApp)