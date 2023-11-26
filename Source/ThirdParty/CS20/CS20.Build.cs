// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class CS20 : ModuleRules
{
	public CS20(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
        bUseRTTI = true;

        if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// Add the import library
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "SynexensSDK.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_core440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_highgui440.lib"));
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_imgcodecs440.lib"));
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_imgproc440.lib"));
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_videoio440.lib"));
			//PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_core440d.lib"));
            //PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_highgui440d.lib"));
            //PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_imgcodecs440d.lib"));
			//PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_imgproc440d.lib"));
            //PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "thirdpart", "lib", "opencv_videoio440d.lib"));
			
            // Add any include paths for the plugin
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "thirdpart", "opencv440"));

            // Ensure that the DLL is staged along with the executable
            /**/
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv/opencv_core440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv/opencv_highgui440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv/opencv_imgcodecs440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv/opencv_imgproc440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv/opencv_videoio440.dll");
			RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/csreconstruction2.0.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/SynexensSDK.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/pthreadVC2.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/SonixCamera.dll");

            // Delay-load the DLL, so we can load it from the right place first
            /**/
            PublicDelayLoadDLLs.Add("csreconstruction2.0.dll");
            PublicDelayLoadDLLs.Add("SynexensSDK.dll");
            PublicDelayLoadDLLs.Add("pthreadVC2.dll");
            PublicDelayLoadDLLs.Add("SonixCamera.dll");
            PublicDelayLoadDLLs.Add("opencv_core440.dll");
            PublicDelayLoadDLLs.Add("opencv_highgui440.dll");
            PublicDelayLoadDLLs.Add("opencv_imgcodecs440.dll");
            PublicDelayLoadDLLs.Add("opencv_imgproc440.dll");
            PublicDelayLoadDLLs.Add("opencv_videoio440.dll");
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
