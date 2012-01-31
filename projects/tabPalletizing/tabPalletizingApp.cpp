#include "GRIPApp.h"
#include "tabPalletizing.h"

extern wxNotebook* tabView;

class PalletizingTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new PalletizingTab(tabView), wxT("PalletizingTab"));
	}
};

IMPLEMENT_APP(PalletizingTabApp)
