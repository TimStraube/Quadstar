from django import forms

class MyForm(forms.Form):
    # startSimulation = forms.CharField(widget=forms.HiddenInput(), initial='Start Simulation')
    # startTraining = forms.CharField(widget=forms.HiddenInput(), initial='Start Training')
    saveModel = forms.CharField(widget=forms.HiddenInput(), initial='Save Model')