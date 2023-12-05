// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class HPS3D : ModuleRules
{
	public HPS3D(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
        bUseRTTI = true;

        if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// Add the import library
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "HPS3D160_SDK.lib"));

            // Add any include paths for the plugin
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));

            // Ensure that the DLL is staged along with the executable
            /**/
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/HPS3D/bin/HPS3D160_SDK.dll");

            // Delay-load the DLL, so we can load it from the right place first
            PublicDelayLoadDLLs.Add("HPS3D160_SDK.dll");
            //*/
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
