// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class Open3D : ModuleRules
{
	public Open3D(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
        bUseRTTI = true;

        if (Target.Platform == UnrealTargetPlatform.Win64)
		{
            // Add the import library
            PublicAdditionalLibraries.AddRange(new string[] {
                Path.Combine(ModuleDirectory, "lib", "glfw3.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_glew.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_liblzf.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_qhull_r.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_qhullcpp.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_rply.lib"),
                Path.Combine(ModuleDirectory, "lib", "Open3D_3rdparty_tinyfiledialogs.lib")
                }
            );

            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "Open3D.lib"));

            // Add any include paths for the plugin
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include", "open3d", "3rdparty"));

            // Ensure that the DLL is staged along with the executable
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/Open3D/bin/Open3D.dll");

            // Delay-load the DLL, so we can load it from the right place first
            PublicDelayLoadDLLs.Add("Open3D.dll");
        }
		/**
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            PublicDelayLoadDLLs.Add(Path.Combine(ModuleDirectory, "Mac", "Release", "libExampleLibrary.dylib"));
            RuntimeDependencies.Add("$(PluginDir)/Source/ThirdParty/Open3D/Mac/Release/libExampleLibrary.dylib");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			string ExampleSoPath = Path.Combine("$(PluginDir)", "Binaries", "ThirdParty", "Open3D", "Linux", "x86_64-unknown-linux-gnu", "libExampleLibrary.so");
			PublicAdditionalLibraries.Add(ExampleSoPath);
			PublicDelayLoadDLLs.Add(ExampleSoPath);
			RuntimeDependencies.Add(ExampleSoPath);
		}
		//*/
	}
}
