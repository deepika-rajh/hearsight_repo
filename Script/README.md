# QCS6490 VM Setup & CI/CD Automation – Full Documentation

## VM Setup Overview

A self-hosted GitHub Actions runner, cross-compilation environment for QCS6490, and automation host for CI/CD, build orchestration, and deployment.

## GCP Details

| Component | Value |
|-----------|-------|
| Machine Type | e2-highcpu-16 (16 vCPUs) |
| CPU Platform | AMD Rome (EPYC 7B12) |
| RAM | 16 GB |
| Disk | 350 GB Persistent SSD |
| OS Image | Ubuntu Minimal 22.04 LTS |
| Public IP | 34.171.18.253 |
| Architecture | x86_64 |

## SSH Access to the VM

Use the following command:

```bash
ssh -i ~/.ssh/gcpcicd cto@34.171.18.253
```

Ensure the VM has your public key added to `~/.ssh/authorized_keys` on the VM.

## GitHub Action Self-Hosted Runner

1. Navigate to your GitHub repository.
2. Go to **Settings → Actions → Runners**.
3. Click "New self-hosted runner" and choose the OS (Linux).
4. GitHub will generate the required setup commands.
5. SSH into the VM and copy–paste all commands to install and configure the runner.
6. Start the service:
   ```bash
   sudo ./svc.sh install
   sudo ./svc.sh start
   ```
7. Confirm the runner status shows **Online** in GitHub.

## SDK Installations on VM

### Qualcomm AI Runtime (QAIRT)

Follow official Qualcomm documentation:
https://docs.qualcomm.com/bundle/publicresource/topics/80-63442-10/linux_setup.html

### Cross Compiling Toolchain eSDK

**Download:**
```bash
wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qimpsdk/qcm6490/x86/qcom-6.6.28-QLI.1.1-Ver.1.1_qim-product-sdk-1.1.3.zip
```

**Install:**
```bash
unzip qcom-6.6.28-QLI.1.1-Ver.1.1_qim-product-sdk-1.1.3.zip
umask a+rx
sh qcom-wayland-x86_64-qcom-multimedia-image-armv8-2a-qcm6490-toolchain-ext-1.0.sh
```

**Set environment:**
```bash
export ESDK_ROOT=<path of installation directory>
cd $ESDK_ROOT
source environment-setup-armv8-2a-qcom-linux
```

**Installed Qualcomm Wayland SDK locations:**
- `/opt/qcom-watland_sdk`
- `/home/cto/qcom-wayland_sdk/`

**To load environment:**
```bash
unset LD_LIBRARY_PATH
source /home/cto/qcom-wayland_sdk/environment-setup-armv8-2a-qcom-linux
# OR
source /opt/qcom-wayland_sdk/environment-setup-armv8-2a-qcom-linux
```

## Project Structure

- **Repository path:** `/home/cto/actions-runner/deployment/hsv2_core/hsv2_core/`
- **Scripts stored in:** `/home/cto/build_script/`

## Multi-Feature Build Script

`build_all_branches.sh` automates:
- Checkout development branch
- Detect feature folders with `build.sh`
- Load environment
- Run `build.sh`
- Validate outputs
- Store binaries in `/home/cto/qcs_builds/`

## CI/CD Workflow (Build & Release)

GitHub Action workflow triggers on development branch push. Runs VM build script, generates `final_development_build.zip`, and uploads as GitHub Release.

## Syncing Workflow Files

VM `sync-scripts-to-vm.yml` automatically updates workflow files in: `/home/cto/build_script/`

### Editing Scripts

**Edit in GitHub:**
- Automatically syncs to VM.

**Direct edit on VM:**
```bash
nano /home/cto/build_script/build_all_branches.sh
```

## Manual Build

```bash
bash /home/cto/build_script/build_all_branches.sh
```

## Deployment

### Destination Server - Advantech

```bash
ssh advantech@192.168.0.104
```

### Deployment Script: `download_and_scp.sh`

- Fetches latest GitHub release
- Downloads ZIP
- SCPs to QCS6490 dev kit at `/home/root/deployment_binary/`
- **Path:** `/home/advantech/github_release_build/`

## Summary

CI/CD, build automation, release generation, VM sync, and deployment fully implemented.
