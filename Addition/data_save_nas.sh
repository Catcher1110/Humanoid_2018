#! /bin/bash
PATH_PREFIX="/home"
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "MAC OS detected"
    PATH_PREFIX="/Users"
elif [[ "$OSTYPE" == "linux"*  ]]; then
    echo "LINUX OS detected
    PATH_PREFIX="/home"
else
    echo "Unsupported OS. Cannot run script""
fi
echo $PATH_PREFIX
PATH_PACKAGE="$PATH_PREFIX/$USER/Repository/dynacore"
echo $PATH_PACKAGE
#PATH_PACKAGE=$(dirname "$(pwd)")
mercury_nodelet="/home/hcrl/ros/mercury_nodelet"

folder_name=$(date +%Y%m%d_%H_%M_%S)
export LATEST_FOLDER_NAME=${folder_name}
echo ${LATEST_FOLDER_NAME}

target_folder="$PATH_PREFIX/$USER/MyCloud/Mercury_Test_2018_06"
data_location=$PATH_PACKAGE
mkdir -p ${target_folder}/${folder_name}
mkdir -p ${target_folder}/${folder_name}/Config

echo "Copying txt files..."
#cp ${mercury_nodelet}/config/* ${target_folder}/${folder_name}/
cp ${data_location}/DynaController/Mercury_Controller/MercuryTestConfig/* ${target_folder}/${folder_name}/Config/
cp ${data_location}/experiment_data/*.txt ${target_folder}/${folder_name}/
cp ${data_location}/experiment_data/*.txt ${data_location}/experiment_data_check/
echo "Finished copying txt files"
# Remove every data for the next experiment
rm $PATH_PACKAGE/experiment_data/*.txt


