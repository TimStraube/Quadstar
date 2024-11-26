import os
import subprocess
from django.shortcuts import render
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt

def landing(request):
    return render(request, "landing.html")
    
@csrf_exempt
def inputUpdate(request):
    if request.method == 'POST':
        
        startSimulation = False

        startTraining = False
        
        stopTraining = False

        saveModel = request.POST.get('saveModel')

        if startSimulation:
            command = ["python3", "/home/quadserver/quadserver/application/quadtest.py"]

            # Start the process
            result = subprocess.run(command, capture_output=True, text=True)

            # Print the output and error (if any)
            print("Output:", result.stdout)
            print("Error:", result.stderr)
            print("Return Code:", result.returncode)

        if startTraining:
            os.system('python3 /home/quadserver/quadserver/application/quadtrain.py')

        return JsonResponse({'status': 'success'})
    return JsonResponse({'status': 'error'})