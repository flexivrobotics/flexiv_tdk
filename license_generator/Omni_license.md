# Applying for Omni license
1. The Omni license is forcibly bound to the user's Ubuntu computer hardware, so before proceeding with the following steps, make sure you have prepared a physical computer with Ubuntu 22.04 and CPU platform x86_64. If you need compatible with other operating systems or CPU architectures, please contact Flexiv.
2. Install net-tools
   ```bash
   sudo apt update
   sudo apt install net-tools
   ```
3. Run the generator under ``flexiv_omni_teleop/omni_license_generator`` on your computer.
   ```bash
   ./generator
   ```
   This will generate a feature_id.txt. Send this file to Flexiv to apply for the Omni license.

   Note: The feature id is a unique identifier for your application and is bound to your computer. You CANNOT use your Omni license and run Omni-Teleop on other devices. Please generate feature_id and use Omni license on the same computer.
4. After received the Omni license, extract the zip package and put the .lic, .signature and .json file to a safe directory. For example, a new folder named ``omni_license`` under your home directory.
5. Parse the path of omni_licenseCfg.json file when construct Omni-Teleop class in your code.