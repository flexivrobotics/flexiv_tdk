# Applying for license
> [!NOTE]
> The license is forcibly bound to the user's computer hardware, so before proceeding with the following steps, make sure you have prepared a physical computer with Ubuntu 20.04/22.04 and CPU platform x86_64. Using virtual machine is highly NOT recommended. If you need to compatible with other operating systems or CPU architectures, please contact Flexiv Ltd.
 
1. Install net-tools
   ```bash
   sudo apt update
   sudo apt install net-tools
   ```
2. Run the generator under ``flexiv_tdk/license_generator`` on your computer.
   ```bash
   ./generator
   ```
   This will generate a feature_id.txt. Send this file to Flexiv to apply for the license.

> [!NOTE]
> The feature id is a unique identifier and is bound to your computer hardware. You CANNOT use your license and run flexiv_tdk on other devices. Please generate feature_id and use license on the same computer.
3. After received the license package, extract the zip package and put the .lic, .signature and .json file to a safe directory. For example, a new folder named ``omni_license`` under your home directory.
4. Parse the path of omni_licenseCfg.json file when construct tdk class in your code.